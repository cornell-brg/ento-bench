# Test script to verify JSON parsing works
cmake_minimum_required(VERSION 3.19)

# Test JSON parsing
set(JSON_CONTENT '{
  "test-group": {
    "reps": 42,
    "verbosity": 2,
    "enable_caches": true
  }
}')

# Test basic JSON access
string(JSON test_reps GET ${JSON_CONTENT} "test-group" "reps")
string(JSON test_verbosity GET ${JSON_CONTENT} "test-group" "verbosity")
string(JSON test_caches GET ${JSON_CONTENT} "test-group" "enable_caches")

message(STATUS "JSON Test Results:")
message(STATUS "  REPS: ${test_reps}")
message(STATUS "  VERBOSITY: ${test_verbosity}")
message(STATUS "  ENABLE_CACHES: ${test_caches}")

# Test error handling
string(JSON nonexistent ERROR_VARIABLE json_error GET ${JSON_CONTENT} "test-group" "nonexistent")
if(json_error)
  message(STATUS "  Error handling works: ${json_error}")
else()
  message(STATUS "  Unexpected: no error for nonexistent key")
endif() 