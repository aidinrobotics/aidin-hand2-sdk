<!-- The dev -> main merge that makes a release.
     Open it with ?template=release.md on the compare URL.

     Title: chore(release): <version>  — for example  chore(release): 0.5.3
     release is not one of the ten types. A release is not a commit but this merge, so the type
     is chore and release is the scope.

     Delete these comments once you have written the sections. -->

## What is in it

<!-- The CHANGELOG section for this version, as it now reads. Paste it rather than linking: a
     reader deciding whether to take this release should not have to open another file. -->

## Version

<!-- Why this number. State which of the four the release carries, because that is what decides
     major, minor or patch:

       a breaking change to the public C++ API or to what is installed
       a new function, type or option
       a fix with no interface change
       nothing user-facing, in which case ask whether it is a release at all

     project(VERSION) in cpp/CMakeLists.txt must already say this number. -->

## Verified

<!-- What was run against this exact tree, not against the branches that fed it.

       ctest 21/21 passed on 22.04 and 24.04
       Built hand type a, b and c for x86_64 and aarch64
       Installed to a clean prefix and ran the example against real hardware -->

## After merging

<!-- Tag the merge commit and push the tag:

       git tag v<version> <merge commit>
       git push origin v<version>

     The CHANGELOG link definitions point at that tag, so without it they resolve to nothing.
     Then open a new Unreleased section on dev. -->
