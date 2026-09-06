# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(usb_host_lib_example)

# ── local fixes to managed components ────────────────────────────────────────
# david-cermak/libssh 0.12.2's mbedTLS-v4 backend was written against an
# mbedTLS 4.x development snapshot, not the 4.0.0 that ESP-IDF v6.0.1 ships:
# it reads pk->psa_type (a field 4.0.0 does not have -- the key type lives in
# pk_info, and ec_family / rsa_padding / rsa_hash_alg carry the rest) and calls
# mbedtls_pk_set_pubkey_from_prv(), which 4.0.0 names
# mbedtls_pk_rsa_set_pubkey_from_prv(). Without the patch the component does
# not compile, and david-cermak/mbedtls_v3_shim is not an escape route: it has
# the same psa_type problem, and 0.12.2 dropped the shim path anyway.
#
# This has to run AFTER project(): that is when the component manager solves
# the manifest and downloads managed_components/, so anything earlier would
# find nothing to patch on a fresh clone and the first build would fail. It is
# still configure time, so the patched source is what gets compiled.
#
# It runs on every configure rather than being applied once by hand because
# managed_components/ is re-fetched whenever the manifest or lock file changes,
# which would silently revert it. The script is idempotent.
execute_process(
    COMMAND "${CMAKE_CURRENT_LIST_DIR}/patches/apply.sh"
    RESULT_VARIABLE _patch_rc
    OUTPUT_VARIABLE _patch_out
    ERROR_VARIABLE  _patch_err)
if(NOT _patch_rc EQUAL 0)
    message(FATAL_ERROR "patches/apply.sh failed:\n${_patch_out}${_patch_err}")
elseif(_patch_out)
    message(STATUS "${_patch_out}")
endif()
