#include "daemon/daemon.hpp"

#include <X11/Xatom.h>
#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xrandr.h>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <memory>
#include <sstream>
#include <thread>

namespace {
int x_error = 0;

int RecordXError(Display*, XErrorEvent* event) {
    x_error = event->error_code;
    return 0;
}

class XConnection {
public:
    XConnection(const XConnection&) = delete;
    XConnection& operator=(const XConnection&) = delete;

    XConnection() : display(XOpenDisplay(nullptr)) {
        if (!display)
            throw std::runtime_error(
                "terminal layout requires an accessible X11 display; no layout fallback is used");

        previous_handler = XSetErrorHandler(RecordXError);
        x_error = 0;
    }

    ~XConnection() {
        XCloseDisplay(display);
        XSetErrorHandler(previous_handler);
    }

    void Check(const char* operation) {
        XSync(display, False);
        const int error = x_error;
        x_error = 0;

        if (error && error != BadWindow) {
            char message[256]{};
            XGetErrorText(display, error, message, sizeof(message));
            throw std::runtime_error(std::string(operation) + ": " + message);
        }
    }

    Display* display;
    XErrorHandler previous_handler;
};

struct Rectangle {
    int x = 0, y = 0, width = 0, height = 0;
};

std::vector<unsigned long> Cardinals(XConnection& x, Window window, const char* name) {
    const Atom property = XInternAtom(x.display, name, False);
    Atom actual_type = None;
    int format = 0;
    unsigned long count = 0, remaining = 0;
    unsigned char* bytes = nullptr;
    const int result = XGetWindowProperty(x.display,
        window,
        property,
        0,
        4096,
        False,
        AnyPropertyType,
        &actual_type,
        &format,
        &count,
        &remaining,
        &bytes);
    std::unique_ptr<unsigned char, decltype(&XFree)> storage(bytes, XFree);
    x.Check(name);

    if (result != Success || actual_type == None || count == 0)
        return {};

    if (format != 32 || remaining)
        throw std::runtime_error(std::string("invalid X11 property: ") + name);

    auto* values = reinterpret_cast<unsigned long*>(bytes);
    return std::vector<unsigned long>(values, values + count);
}

std::string Text(XConnection& x, Window window, const char* name) {
    Atom actual_type = None;
    int format = 0;
    unsigned long count = 0, remaining = 0;
    unsigned char* bytes = nullptr;
    const int result = XGetWindowProperty(x.display,
        window,
        XInternAtom(x.display, name, False),
        0,
        4096,
        False,
        AnyPropertyType,
        &actual_type,
        &format,
        &count,
        &remaining,
        &bytes);
    std::unique_ptr<unsigned char, decltype(&XFree)> storage(bytes, XFree);
    x.Check(name);

    if (result != Success || actual_type == None || format != 8 || !bytes)
        return {};

    return std::string(reinterpret_cast<char*>(bytes), count);
}

Window FindWindow(XConnection& x,
    Window parent,
    const char* property,
    const std::string& expected,
    int depth = 0) {
    XWindowAttributes attributes{};
    const bool live = XGetWindowAttributes(x.display, parent, &attributes);
    x.Check("window attributes");

    if (!live)
        return None;

    if (attributes.map_state == IsViewable && Text(x, parent, property) == expected)
        return parent;

    if (depth >= 4)
        return None;

    Window root = None, ancestor = None, *children = nullptr;
    unsigned count = 0;
    const bool queried = XQueryTree(x.display, parent, &root, &ancestor, &children, &count);
    std::unique_ptr<Window, decltype(&XFree)> storage(children, XFree);
    x.Check("window tree");

    if (!queried)
        return None;

    for (unsigned i = 0; i < count; ++i) {
        const Window found = FindWindow(x, children[i], property, expected, depth + 1);

        if (found != None)
            return found;
    }

    return None;
}

Window WaitForWindow(
    XConnection& x, const char* property, const std::string& expected, int timeout_ms) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);

    do {
        const Window window = FindWindow(x, DefaultRootWindow(x.display), property, expected);

        if (window != None)
            return window;

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    } while (std::chrono::steady_clock::now() < deadline);

    const std::string advice =
        std::strcmp(property, "_NET_WM_NAME") == 0
            ? "; for an embedded terminal set terminal_layout.include_daemon: false explicitly"
            : "; GNOME must publish an X11 window role; native Wayland placement is unsupported";
    throw std::runtime_error("terminal layout could not find its X11 window: " + expected + advice);
}

void SendEvent(XConnection& x,
    Window window,
    const char* type,
    long a,
    long b = 0,
    long c = 0,
    long d = 0,
    long e = 0) {
    XEvent event{};
    event.xclient.type = ClientMessage;
    event.xclient.window = window;
    event.xclient.message_type = XInternAtom(x.display, type, False);
    event.xclient.format = 32;
    event.xclient.data.l[0] = a;
    event.xclient.data.l[1] = b;
    event.xclient.data.l[2] = c;
    event.xclient.data.l[3] = d;
    event.xclient.data.l[4] = e;

    if (!XSendEvent(x.display,
            DefaultRootWindow(x.display),
            False,
            SubstructureRedirectMask | SubstructureNotifyMask,
            &event))
        throw std::runtime_error("window manager rejected a terminal layout request");

    x.Check(type);
}

void PlaceWindow(XConnection& x, Window window, const Rectangle& slot) {
    const bool managed =
        !Cardinals(x, DefaultRootWindow(x.display), "_NET_SUPPORTING_WM_CHECK").empty();

    if (managed) {
        const auto supported = Cardinals(x, DefaultRootWindow(x.display), "_NET_SUPPORTED");
        const Atom move = XInternAtom(x.display, "_NET_MOVERESIZE_WINDOW", False);

        if (std::find(supported.begin(), supported.end(), move) == supported.end())
            throw std::runtime_error("X11 window manager does not support _NET_MOVERESIZE_WINDOW");

        SendEvent(x,
            window,
            "_NET_WM_STATE",
            0,
            XInternAtom(x.display, "_NET_WM_STATE_MAXIMIZED_VERT", False),
            XInternAtom(x.display, "_NET_WM_STATE_MAXIMIZED_HORZ", False),
            2);
        SendEvent(x,
            window,
            "_NET_WM_STATE",
            0,
            XInternAtom(x.display, "_NET_WM_STATE_FULLSCREEN", False),
            0,
            2);
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    auto stable_since = std::chrono::steady_clock::time_point{};

    do {
        const auto extents = Cardinals(x, window, "_NET_FRAME_EXTENTS");
        const int left = extents.size() == 4 ? int(extents[0]) : 0;
        const int right = extents.size() == 4 ? int(extents[1]) : 0;
        const int top = extents.size() == 4 ? int(extents[2]) : 0;
        const int bottom = extents.size() == 4 ? int(extents[3]) : 0;
        int width = slot.width - left - right;
        int height = slot.height - top - bottom;
        XSizeHints hints{};
        long supplied = 0;
        const bool have_hints = XGetWMNormalHints(x.display, window, &hints, &supplied);
        x.Check("terminal size hints");

        if (have_hints) {
            if ((hints.flags & PMinSize) && (width < hints.min_width || height < hints.min_height))
                throw std::runtime_error(
                    "terminal minimum size exceeds its layout tile; select a larger monitor or fewer columns");

            if (hints.flags & PResizeInc) {
                const int base_width = (hints.flags & PBaseSize)
                                           ? hints.base_width
                                           : ((hints.flags & PMinSize) ? hints.min_width : 0);
                const int base_height = (hints.flags & PBaseSize)
                                            ? hints.base_height
                                            : ((hints.flags & PMinSize) ? hints.min_height : 0);

                if (hints.width_inc > 0 && width > base_width)
                    width = base_width + (width - base_width) / hints.width_inc * hints.width_inc;

                if (hints.height_inc > 0 && height > base_height)
                    height =
                        base_height + (height - base_height) / hints.height_inc * hints.height_inc;
            }
        }

        if (width <= 0 || height <= 0)
            throw std::runtime_error("terminal decorations exceed the layout tile");

        if (managed)
            SendEvent(x,
                window,
                "_NET_MOVERESIZE_WINDOW",
                StaticGravity | (15 << 8) | (2 << 12),
                slot.x + left,
                slot.y + top,
                width,
                height);
        else {
            XMoveResizeWindow(x.display, window, slot.x + left, slot.y + top, width, height);
            x.Check("terminal move/resize");
        }

        XWindowAttributes actual{};
        Window child = None;
        int client_x = 0, client_y = 0;
        const bool have_attributes = XGetWindowAttributes(x.display, window, &actual);
        const bool translated = XTranslateCoordinates(
            x.display, window, DefaultRootWindow(x.display), 0, 0, &client_x, &client_y, &child);
        x.Check("terminal placement verification");

        bool maximized = false;

        if (managed) {
            const auto states = Cardinals(x, window, "_NET_WM_STATE");

            for (const char* name : {"_NET_WM_STATE_MAXIMIZED_VERT",
                     "_NET_WM_STATE_MAXIMIZED_HORZ",
                     "_NET_WM_STATE_FULLSCREEN"}) {
                const auto state = XInternAtom(x.display, name, False);

                if (std::find(states.begin(), states.end(), state) != states.end()) {
                    maximized = true;
                    SendEvent(x, window, "_NET_WM_STATE", 0, state, 0, 2);
                }
            }
        }

        if (!maximized && have_attributes && translated && client_x - left >= slot.x &&
            client_y - top >= slot.y && client_x + actual.width + right <= slot.x + slot.width &&
            client_y + actual.height + bottom <= slot.y + slot.height &&
            actual.width >= width - 2 && actual.height >= height - 2) {
            const auto now = std::chrono::steady_clock::now();

            if (stable_since == std::chrono::steady_clock::time_point{})
                stable_since = now;

            // GTK and the WM can apply initial geometry after the first mapped frame.
            if (now - stable_since >= std::chrono::milliseconds(250)) {
                XRaiseWindow(x.display, window);
                x.Check("terminal raise");
                return;
            }
        } else
            stable_since = std::chrono::steady_clock::time_point{};

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    } while (std::chrono::steady_clock::now() < deadline);

    throw std::runtime_error("window manager did not honor the non-overlapping terminal layout");
}
}

void Daemon::PlanTerminalLayout(std::vector<ProcessSpec>& specs,
    const std::string& monitor,
    int columns,
    int gap_px,
    int margin_px,
    double zoom,
    bool include_daemon) {
    if (columns < 1 || columns > 8 || gap_px < 0 || gap_px > 200 || margin_px < 0 ||
        margin_px > 200 || !std::isfinite(zoom) || zoom < .5 || zoom > 2 || monitor.empty())
        throw std::runtime_error(
            "invalid terminal_layout: columns 1..8, gap/margin 0..200 px, zoom .5..2, monitor required");

    XConnection x;
    int count = 0;
    XRRMonitorInfo* monitors =
        XRRGetMonitors(x.display, DefaultRootWindow(x.display), True, &count);
    std::unique_ptr<XRRMonitorInfo, decltype(&XRRFreeMonitors)> storage(monitors, XRRFreeMonitors);
    x.Check("X11 monitors");

    if (!monitors || count <= 0)
        throw std::runtime_error("terminal layout requires XRandR monitor information");

    std::vector<std::string> names;
    int primary = 0;

    for (int i = 0; i < count; ++i) {
        char* name = XGetAtomName(x.display, monitors[i].name);
        names.emplace_back(name ? name : "");
        XFree(name);

        if (monitors[i].primary)
            primary = i;
    }

    int selected = -1;

    if (monitor == "primary")
        selected = primary;
    else if (monitor == "auto") {
        selected = primary;

        for (int i = 0; i < count; ++i)
            if (i != primary) {
                selected = i;
                break;
            }
    } else {
        const auto found = std::find(names.begin(), names.end(), monitor);

        if (found != names.end())
            selected = int(found - names.begin());
    }

    if (selected < 0)
        throw std::runtime_error("configured terminal monitor is unavailable: " + monitor);

    Rectangle area{monitors[selected].x,
        monitors[selected].y,
        monitors[selected].width,
        monitors[selected].height};
    const auto workareas = Cardinals(x, DefaultRootWindow(x.display), "_NET_WORKAREA");

    if (workareas.empty())
        WriteTerminalLog(
            "[layout] X11 work area not advertised; using monitor bounds (no panel reservation)\n");
    else {
        const auto desktops = Cardinals(x, DefaultRootWindow(x.display), "_NET_CURRENT_DESKTOP");
        const auto desktop = desktops.empty() ? 0 : desktops[0];

        if (workareas.size() < 4 || desktop >= workareas.size() / 4)
            throw std::runtime_error("invalid X11 desktop work area");

        const auto index = 4 * desktop;
        const int right =
            std::min(area.x + area.width, int(workareas[index] + workareas[index + 2]));
        const int bottom =
            std::min(area.y + area.height, int(workareas[index + 1] + workareas[index + 3]));
        area.x = std::max(area.x, int(workareas[index]));
        area.y = std::max(area.y, int(workareas[index + 1]));
        area.width = right - area.x;
        area.height = bottom - area.y;
    }

    area.x += margin_px;
    area.y += margin_px;
    area.width -= 2 * margin_px;
    area.height -= 2 * margin_px;
    const bool caller = include_daemon && ::isatty(STDOUT_FILENO);
    std::size_t terminals = caller ? 1 : 0;

    for (const auto& spec : specs)
        terminals += !spec.terminal_title.empty();

    if (!terminals)
        return;

    const int row_columns = std::min<int>(columns, terminals);
    const int rows = (int(terminals) + row_columns - 1) / row_columns;
    const int height = (area.height - gap_px * (rows - 1)) / rows;

    if (height < 360 || (area.width - gap_px * (row_columns - 1)) / row_columns < 600)
        throw std::runtime_error(
            "terminal work area is too small for readable tiles (minimum 600x360 px); change monitor or columns");

    std::ostringstream layout_log;
    layout_log << "[layout] monitor=" << names[selected] << " policy=" << monitor
               << " area=" << area.width << 'x' << area.height << '+' << area.x << '+' << area.y
               << " windows=" << terminals << " columns=" << row_columns << " gap=" << gap_px
               << " zoom=" << zoom << " daemon=" << caller << '\n';
    WriteTerminalLog(layout_log.str());
    const auto tile = [&](std::size_t index) {
        const int row = int(index) / row_columns;
        const int in_row = std::min<int>(row_columns, terminals - row * row_columns);
        const int width = (area.width - gap_px * (in_row - 1)) / in_row;
        const int column = int(index) % row_columns;
        return Rectangle{
            area.x + column * (width + gap_px), area.y + row * (height + gap_px), width, height};
    };
    std::size_t index = 0;

    if (caller) {
        const std::string title = "QHRR05-DAEMON [" + std::to_string(::getpid()) + "]";
        const auto escape = "\033]0;" + title + '\a';
        std::cout << escape << std::flush;
        const Window window = WaitForWindow(x, "_NET_WM_NAME", title, 3000);
        PlaceWindow(x, window, tile(index++));
    }

    for (auto& spec : specs) {
        if (spec.terminal_title.empty())
            continue;

        const auto position = tile(index++);
        spec.terminal_role = "qhrr05-" + std::to_string(::getpid()) + '-' + spec.name;
        spec.terminal_x = position.x;
        spec.terminal_y = position.y;
        spec.terminal_width = position.width;
        spec.terminal_height = position.height;
        spec.terminal_zoom = zoom;
    }
}

void Daemon::PlaceTerminalWindow(const ProcessSpec& spec, int timeout_ms) {
    XConnection x;
    const Window window = WaitForWindow(x, "WM_WINDOW_ROLE", spec.terminal_role, timeout_ms);
    PlaceWindow(x,
        window,
        Rectangle{spec.terminal_x, spec.terminal_y, spec.terminal_width, spec.terminal_height});
    std::ostringstream layout_log;
    layout_log << "[layout] placed " << spec.name << " " << spec.terminal_width << 'x'
               << spec.terminal_height << '+' << spec.terminal_x << '+' << spec.terminal_y << '\n';
    WriteTerminalLog(layout_log.str());
}
