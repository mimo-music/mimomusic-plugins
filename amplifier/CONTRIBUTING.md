# Contributing

Thank you for helping! Bug reports, measurements, presets and code are
all welcome.

## Bug reports and ideas

Open an issue. For crashes, please include your host (e.g. Ardour 9.x),
operating system, plugin format (LV2/VST3/AU) and, if you have one, the
backtrace or crash log.

## Code contributions

1. **Sign the Contributor License Agreement (CLA) once.** When you open
   your first pull request, the CLA Assistant bot will ask you to sign
   [CLA.md](CLA.md). We can only merge contributions from people who have
   signed.
2. Keep each pull request focused on one change, and describe what it
   does and how you tested it.
3. Run the project's own checks before submitting:
   `./build_and_install.sh --no-install --test` must build without new
   warnings, and all tests must pass.
4. In the pull request description, explain what problem the change
   solves, what changed and why, and how you verified it.

## Why is there a CLA?

These plugins are published as free software under GPL-3.0-or-later. You
can use, study, change and share them under those terms.

Mettronic GmbH, which publishes MimoMusic, also builds this code into
its own products, including embedded hardware, and may license it to
others under commercial terms. The CLA gives Mettronic the right to do
that with your contribution as well. You keep the copyright on your work,
and every open-source release that contains your contribution stays
available under GPL-3.0-or-later (or another OSI-approved license) --
see section 4 of the CLA.

If you don't want to sign, you are still welcome to open issues, and to
fork and change the code under the GPL.
