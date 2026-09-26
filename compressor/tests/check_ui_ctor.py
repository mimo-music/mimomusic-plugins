#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-or-later
# SPDX-FileCopyrightText: 2026 Mettronic GmbH
"""Static check for a UI class: every pointer member the header declares as `= nullptr` must be assigned with
`new` exactly once, at the TOP LEVEL of the constructor body (brace depth 1) -- not inside a lambda / nested block,
and not missing. Written after a spliced edit put `btnDots_ = new SimpleButton(...)` inside another button's click
lambda: the code was valid C++, so no compile check noticed, and the constructor then left btnDots_ null and
layout() crashed Ardour the moment the editor opened.

  python3 tests/check_ui_ctor.py gui/UIContour.hpp gui/UIContour.cpp UIContour [more: hpp cpp Class ...]
"""
import re
import sys


def check(hpp, cpp, cls):
    h = open(hpp).read()
    members = re.findall(r'^\s*(?:DGL_NAMESPACE::)?[A-Za-z_:]+\s*\*\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*nullptr\s*;', h, re.M)
    src = open(cpp).read()
    m = re.search(r'\b' + cls + r'::' + cls + r'\s*\([^)]*\)[^{]*\{', src)
    if not m:
        print("%s: constructor not found" % cls); return 1
    i = m.end() - 1
    depth = 0
    j = i
    while j < len(src):
        c = src[j]
        if c == '{': depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0: break
        j += 1
    body = src[i:j + 1]
    # walk the body tracking brace depth per position
    depth = 0
    depth_at = []
    for ch in body:
        if ch == '{': depth += 1
        depth_at.append(depth)
        if ch == '}': depth -= 1
    bad = 0
    for name in members:
        hits = [(mm.start(), depth_at[mm.start()]) for mm in re.finditer(r'(?<![A-Za-z0-9_>])' + re.escape(name) + r'\s*=\s*new\b', body)]
        if not hits:
            print("  NOT CREATED in the constructor: %s" % name); bad += 1
        elif len(hits) > 1:
            print("  created %d times: %s" % (len(hits), name)); bad += 1
        elif hits[0][1] != 1:
            print("  created at brace depth %d (inside a lambda/nested block?): %s" % (hits[0][1], name)); bad += 1
        else:
            # used (name->...) at the constructor's own top level BEFORE it is created? (lambdas run later, so only depth 1 counts)
            for um in re.finditer(r'(?<![A-Za-z0-9_>])' + re.escape(name) + r'\s*->', body):
                if um.start() < hits[0][0] and depth_at[um.start()] == 1:
                    print("  used before it is created (constructor top level): %s" % name); bad += 1; break
    print("%s: %d pointer members checked, %d problem(s)" % (cls, len(members), bad))
    return bad


if __name__ == '__main__':
    a = sys.argv[1:]
    if len(a) < 3 or len(a) % 3:
        print(__doc__); sys.exit(2)
    total = 0
    for k in range(0, len(a), 3):
        total += check(a[k], a[k + 1], a[k + 2])
    sys.exit(1 if total else 0)
