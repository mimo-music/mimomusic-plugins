// SPDX-License-Identifier: GPL-3.0-or-later
// SPDX-FileCopyrightText: 2026 Mettronic GmbH

// tests/gui_stub/NanoVG.hpp
//
// A minimal stand-in for DPF's "NanoVG.hpp" (a copy of ../compressor's) so a NanoVG
// widget can be compiled and DRIVEN headlessly
// -- no OpenGL, no X11, no window: drawing calls are no-ops, and the mouse/motion
// event structs are just enough to feed onMouse()/onMotion(). Only the mouse LOGIC
// of the widget is under test through it (hit-testing, gesture state machines);
// nothing here renders anything. Put this directory first on the include path:
//   g++ -Itests/gui_stub -Igui/dyneq2 ... tests/dyneq2_list_sim.cpp gui/dyneq2/DynEqBandListWidget2.cpp
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <cstring>

#define START_NAMESPACE_DGL namespace DGL {
#define END_NAMESPACE_DGL }

START_NAMESPACE_DGL

template <typename T> struct Point
{
    T x{}, y{};
    Point() = default;
    Point(T px, T py) : x(px), y(py) {}
    T getX() const { return x; }
    T getY() const { return y; }
};

struct Color
{
    float red = 0, green = 0, blue = 0, alpha = 1;
    Color() = default;
    Color(int r, int g, int b, float a = 1.0f) : red(float(r)), green(float(g)), blue(float(b)), alpha(a) {}
    Color(float r, float g, float b, float a) : red(r), green(g), blue(b), alpha(a) {}
    Color withAlpha(float a) const { Color c = *this; c.alpha = a; return c; }
};

enum Align { ALIGN_LEFT = 1, ALIGN_CENTER = 2, ALIGN_RIGHT = 4, ALIGN_TOP = 8, ALIGN_MIDDLE = 16, ALIGN_BOTTOM = 32, ALIGN_BASELINE = 64 };
inline int operator|(Align a, Align b) { return int(a) | int(b); }

enum Modifier { kModifierShift = 1, kModifierControl = 2, kModifierAlt = 4, kModifierSuper = 8 };
enum { kMouseButtonRight = 3 };
constexpr const char* NANOVG_DEJAVU_SANS_TTF = "dejavu";

struct MouseEvent { Point<double> pos; uint32_t button = 1; bool press = false; uint32_t mod = 0; };
struct MotionEvent { Point<double> pos; uint32_t mod = 0; };
struct ScrollEvent { Point<double> pos; Point<double> delta; uint32_t mod = 0; };

struct Paint {};
template <typename T> struct Rectangle { T x{}, y{}, w{}, h{}; T getWidth() const { return w; } T getHeight() const { return h; } };

struct FakeWindow { void focus() {} unsigned getWidth() const { return 800; } };

struct Widget { virtual ~Widget() = default; };
// Widget constructors take the editor as a NanoTopLevelWidget* (shared NanoVG
// context, see the UI constructors); in the stub any Widget will do.
using NanoTopLevelWidget = Widget;

// The NanoSubWidget surface the widgets use. Size is settable by the test.
struct NanoSubWidget : Widget
{
    explicit NanoSubWidget(Widget*) {}
    void save() {}
    void restore() {}
    void translate(float, float) {}
    int getAbsoluteX() const { return 0; }
    int getAbsoluteY() const { return 0; }
    // sizing / repaint
    double w_ = 800, h_ = 600;
    unsigned getWidth() const { return unsigned(w_); }
    unsigned getHeight() const { return unsigned(h_); }
    void repaint() {}
    FakeWindow& getWindow() const { static FakeWindow fw; return fw; }
    // pixelRatio() == 1: the fake window and top-level widget are the same width.
    struct FakeTop { unsigned getWidth() const { return 800; } };
    const FakeTop* getTopLevelWidget() const { static FakeTop t; return &t; }
    void loadSharedResources() {}
    // drawing: all no-ops
    void beginPath() {}
    void rect(float, float, float, float) {}
    void roundedRect(float, float, float, float, float) {}
    void circle(float, float, float) {}
    void moveTo(float, float) {}
    void lineTo(float, float) {}
    void fill() {}
    void stroke() {}
    void fillColor(const Color&) {}
    void strokeColor(const Color&) {}
    void strokeWidth(float) {}
    void fontSize(float) {}
    void fontFace(const char*) {}
    void textAlign(int) {}
    void text(float, float, const char*, const char*) {}
    // text measuring: a fixed-width fake (only layout-independent logic is tested)
    float textBounds(float, float, const char* s, const char*, Rectangle<float>& r) { r = { 0, 0, 6.0f * float(s ? std::string(s).size() : 0), 12.0f }; return r.w; }
    void scissor(float, float, float, float) {}
    void resetScissor() {}
    void fillPaint(const Paint&) {}
    template <typename... A> Paint linearGradient(A&&...) { return Paint(); }
    virtual void onNanoDisplay() {}
    virtual bool onMouse(const MouseEvent&) { return false; }
    virtual bool onMotion(const MotionEvent&) { return false; }
    virtual bool onScroll(const ScrollEvent&) { return false; }
};

END_NAMESPACE_DGL
