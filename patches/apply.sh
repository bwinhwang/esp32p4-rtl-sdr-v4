#!/bin/sh
# Applies the local fixes to managed components. Idempotent: a patch that is
# already in place is skipped, so this is safe to run on every build.
#
# managed_components/ is re-fetched by the IDF component manager whenever the
# manifest or the lock file changes, which silently reverts anything patched
# there -- hence running this from the top-level CMakeLists rather than once by
# hand. See the note there for what the patch is for.
set -e
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
target="$root/managed_components/david-cermak__libssh/libssh-mirror/src/mbedcrypto_v4.c"

[ -f "$target" ] || exit 0                                   # component not fetched yet
grep -q 'mbedtls_pk_rsa_set_pubkey_from_prv' "$target" && exit 0   # already applied

command -v patch >/dev/null 2>&1 || {
    echo "patches/apply.sh: 'patch' not found -- libssh will not build against" >&2
    echo "  mbedTLS 4.0.0 without it. Install patch, or apply" >&2
    echo "  patches/libssh-0.12.2-mbedtls-4.0.0.patch by hand." >&2
    exit 1
}

patch -p0 -s -d "$(dirname "$target")" < "$root/patches/libssh-0.12.2-mbedtls-4.0.0.patch"
echo "applied patches/libssh-0.12.2-mbedtls-4.0.0.patch"
