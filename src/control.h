#pragma once

// Run/save flags shared by the windows, the main loop and the signal handler.

#include <atomic>
#include <csignal>
#include <cstring>

namespace rtabmap_minimal {

static_assert(std::atomic<bool>::is_always_lock_free,
		"the signal handler below may only touch lock-free atomics");
inline std::atomic<bool> g_stop(false);
inline std::atomic<bool> g_save(false);

inline void onSignal(int) { g_stop = true; }

inline void installSignalHandler()
{
	struct sigaction action;
	std::memset(&action, 0, sizeof(action));
	action.sa_handler = onSignal;
	sigaction(SIGINT, &action, nullptr);
	sigaction(SIGTERM, &action, nullptr);
}
} // namespace rtabmap_minimal
