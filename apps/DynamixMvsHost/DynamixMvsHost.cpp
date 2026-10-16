/*
 * DynamixMvsHost.cpp
 *
 * Worker process for DynamixMapper: runs one OpenMVS app per process - its own main(), compiled
 * in here unchanged (see CMakeLists.txt) - and talks to the caller over two named pipes instead
 * of the console. The caller gets every log line, progress and SfM live frame as a message, can
 * ask the run to stop, and learns how it ended; a crash or an exit() inside OpenMVS still only
 * ends this process.
 *
 * Usage: DynamixMvsHost --pipe <name>
 *   connects to \\.\pipe\<name>-to-host (read) and \\.\pipe\<name>-from-host (write), both
 *   created by the caller, then serves exactly one request.
 *
 * Message frame, both directions: uint32 length (of what follows), uint8 type, payload.
 * Strings are in the process ANSI code page, as OpenMVS takes them.
 *   caller -> host  'R' run      app name and its arguments, NUL-separated (argv without argv[0])
 *                   'C' cancel   no payload
 *   host -> caller  'L' log      one log message
 *                   'P' progress uint64 done, uint64 total, step name
 *                   'F' frame    uint32 image ID, live snapshot text (the live_frame_<id>.txt content)
 *                   'E' end      int32 the app's exit code, uint8 1 if a cancel was requested
 */

#include "../../libs/MVS/Common.h"
#include "../../libs/SFM/Common.h"
#include "../../libs/SFM/Scene.h"
#include <windows.h>
#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace MVS;

// the hosted apps' entry points: their main(), renamed at compile time
int DensifyPointCloud_main(int argc, LPCTSTR* argv);
int CreateStructure_main(int argc, LPCTSTR* argv);
int ReconstructMesh_main(int argc, LPCTSTR* argv);
int RefineMesh_main(int argc, LPCTSTR* argv);
int TextureMesh_main(int argc, LPCTSTR* argv);


namespace {

typedef int (*AppMain)(int, LPCTSTR*);
struct HostedApp { const char* name; AppMain main; };
const HostedApp g_apps[] = {
	{ "DensifyPointCloud", &DensifyPointCloud_main },
	{ "CreateStructure",   &CreateStructure_main },
	{ "ReconstructMesh",   &ReconstructMesh_main },
	{ "RefineMesh",        &RefineMesh_main },
	{ "TextureMesh",       &TextureMesh_main },
};

// a caller that vanished gets this long to let a canceled run stop on its own
const DWORD ORPHAN_GRACE_MS = 10000;

HANDLE g_pipeIn(INVALID_HANDLE_VALUE);
HANDLE g_pipeOut(INVALID_HANDLE_VALUE);
std::mutex g_writeLock;
std::atomic<bool> g_bCancelRequested(false);

bool WriteAll(HANDLE h, const void* data, DWORD size)
{
	const char* p = static_cast<const char*>(data);
	while (size > 0) {
		DWORD written(0);
		if (!WriteFile(h, p, size, &written, NULL) || written == 0)
			return false;
		p += written;
		size -= written;
	}
	return true;
}

bool ReadAll(HANDLE h, void* data, DWORD size)
{
	char* p = static_cast<char*>(data);
	while (size > 0) {
		DWORD read(0);
		if (!ReadFile(h, p, size, &read, NULL) || read == 0)
			return false;
		p += read;
		size -= read;
	}
	return true;
}

// Sends one message; callable from any thread (log, progress and live frames come from worker threads).
bool Send(char type, const void* payload, size_t size)
{
	const uint32_t length((uint32_t)size + 1);
	std::lock_guard<std::mutex> lock(g_writeLock);
	if (g_pipeOut == INVALID_HANDLE_VALUE)
		return false;
	return WriteAll(g_pipeOut, &length, sizeof(length)) &&
		WriteAll(g_pipeOut, &type, 1) &&
		(size == 0 || WriteAll(g_pipeOut, payload, (DWORD)size));
}

bool Receive(char& type, std::string& payload)
{
	uint32_t length(0);
	if (!ReadAll(g_pipeIn, &length, sizeof(length)) || length == 0)
		return false;
	if (!ReadAll(g_pipeIn, &type, 1))
		return false;
	payload.resize(length - 1);
	return length == 1 || ReadAll(g_pipeIn, &payload[0], length - 1);
}

void SendLog(const char* text, size_t len)
{
	// the logger ends each message with a new-line, which a message does not need
	while (len > 0 && (text[len-1] == '\n' || text[len-1] == '\r'))
		--len;
	Send('L', text, len);
}

class LogForwarder
{
public:
	void Record(const String& msg) { SendLog(msg.c_str(), msg.length()); }
};
LogForwarder g_logForwarder;

void ForwardProgress(LPCTSTR msg, size_t done, size_t total)
{
	// once canceled, a step's progress is torn down as if complete ("36/36"): not passed on,
	// so the caller never shows a canceled step as finished
	if (g_bCancelRequested)
		return;
	std::string payload(2*sizeof(uint64_t), '\0');
	const uint64_t values[2] = { (uint64_t)done, (uint64_t)total };
	memcpy(&payload[0], values, sizeof(values));
	payload += msg;
	Send('P', payload.data(), payload.size());
}

void ForwardLiveFrame(uint32_t imageID, const char* snapshot)
{
	std::string payload(sizeof(uint32_t), '\0');
	memcpy(&payload[0], &imageID, sizeof(imageID));
	payload += snapshot;
	Send('F', payload.data(), payload.size());
}

void RequestCancel()
{
	g_bCancelRequested = true;
	Util::RequestCancel(true);
}

// Serves cancel requests while the app runs. A broken pipe means the caller is gone: the run is
// canceled and, if it does not stop by itself (most stages have no cancel checkpoints), ended.
void ReadCancelRequests()
{
	char type;
	std::string payload;
	while (Receive(type, payload)) {
		if (type == 'C')
			RequestCancel();
	}
	RequestCancel();
	Sleep(ORPHAN_GRACE_MS);
	TerminateProcess(GetCurrentProcess(), 3);
}

HANDLE OpenPipe(const std::string& name, DWORD access)
{
	const std::string path("\\\\.\\pipe\\" + name);
	for (int attempt = 0; attempt < 50; ++attempt) {
		const HANDLE h(CreateFileA(path.c_str(), access, 0, NULL, OPEN_EXISTING, 0, NULL));
		if (h != INVALID_HANDLE_VALUE)
			return h;
		if (GetLastError() != ERROR_PIPE_BUSY && GetLastError() != ERROR_FILE_NOT_FOUND)
			break;
		Sleep(100);
	}
	return INVALID_HANDLE_VALUE;
}

} // unnamed namespace


int main(int argc, char** argv)
{
	std::string pipeName;
	for (int i = 1; i+1 < argc; ++i)
		if (strcmp(argv[i], "--pipe") == 0)
			pipeName = argv[i+1];
	if (pipeName.empty()) {
		fprintf(stderr, "usage: DynamixMvsHost --pipe <name>\n");
		return 10;
	}
	g_pipeIn = OpenPipe(pipeName + "-to-host", GENERIC_READ);
	g_pipeOut = OpenPipe(pipeName + "-from-host", GENERIC_WRITE);
	if (g_pipeIn == INVALID_HANDLE_VALUE || g_pipeOut == INVALID_HANDLE_VALUE) {
		fprintf(stderr, "DynamixMvsHost: cannot connect to pipe '%s' (error %lu)\n", pipeName.c_str(), GetLastError());
		return 11;
	}

	// the request: app name, then its arguments
	char type;
	std::string request;
	if (!Receive(type, request) || type != 'R') {
		fprintf(stderr, "DynamixMvsHost: no run request received\n");
		return 12;
	}
	std::vector<std::string> args;
	for (size_t start = 0; start <= request.size(); ) {
		size_t end(request.find('\0', start));
		if (end == std::string::npos)
			end = request.size();
		args.emplace_back(request, start, end - start);
		start = end + 1;
	}
	AppMain appMain(NULL);
	for (const HostedApp& app: g_apps)
		if (args.front() == app.name)
			appMain = app.main;
	if (appMain == NULL) {
		const std::string msg("error: DynamixMvsHost does not host '" + args.front() + "'");
		SendLog(msg.c_str(), msg.length());
		const int32_t exitCode(EXIT_FAILURE);
		char end[sizeof(int32_t) + 1] = {};
		memcpy(end, &exitCode, sizeof(exitCode));
		Send('E', end, sizeof(end));
		return EXIT_FAILURE;
	}

	// listen before the app starts: its own OPEN_LOG() keeps listeners that are already there
	OPEN_LOG();
	GET_LOG().RegisterListener(DELEGATEBINDCLASS(Log::ClbkRecordMsg, &LogForwarder::Record, &g_logForwarder));
	Util::SetProgressListener(&ForwardProgress);
	SFM::SetLiveFrameListener(&ForwardLiveFrame);
	std::thread(ReadCancelRequests).detach();

	// run the app as if launched with these arguments (argv[0] is its name)
	std::vector<LPCTSTR> appArgv;
	for (const std::string& arg: args)
		appArgv.push_back(arg.c_str());
	appArgv.push_back(NULL);
	const int32_t exitCode(appMain((int)args.size(), appArgv.data()));

	Util::SetProgressListener(NULL);
	SFM::SetLiveFrameListener(NULL);
	char end[sizeof(int32_t) + 1];
	memcpy(end, &exitCode, sizeof(exitCode));
	end[sizeof(int32_t)] = g_bCancelRequested ? 1 : 0;
	Send('E', end, sizeof(end));
	{
		std::lock_guard<std::mutex> lock(g_writeLock);
		FlushFileBuffers(g_pipeOut);
	}
	// end here, not through static destructors: the cancel reader may still be blocked on the pipe
	ExitProcess((UINT)exitCode);
}
/*----------------------------------------------------------------*/
