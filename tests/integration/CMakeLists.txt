## Integration tier: composed algorithm pipelines validated against the
## simulation tier (see docs/LESSONS.md + the validation-ladder notes) —
## unit tests pin each algorithm; these pin the compositions.
##
## Labels:
##   integration — composed closed-loop scenarios; run on every PR
##   campaign    — Monte-Carlo robustness with plant-model mismatch;
##                 excluded from PR ctest (-LE campaign), run nightly.
##                 Campaign binaries still BUILD on PRs so they cannot rot.

add_executable(test_pipeline_diffdrive test_pipeline_diffdrive.cpp)
target_link_libraries(test_pipeline_diffdrive
    estimation mppi shield sim gtest gtest_main)
gtest_discover_tests(test_pipeline_diffdrive PROPERTIES LABELS integration)

add_executable(campaign_diffdrive campaign_diffdrive.cpp)
target_link_libraries(campaign_diffdrive
    mppi shield sim gtest gtest_main)
gtest_discover_tests(campaign_diffdrive PROPERTIES LABELS campaign)
