#!/bin/bash

# build_with_progress.sh - Colored progress builder for benchmark groups
# Usage: ./build_with_progress.sh GROUP_NAME "target1 target2 target3" [BUILD_DIR] [CONFIG_STR]

GROUP_NAME="$1"
TARGET_LIST="$2"
BUILD_DIR="${3:-$(pwd)}"
CONFIG_STR="${4:-}"

# Validate inputs
if [ -z "$GROUP_NAME" ] || [ -z "$TARGET_LIST" ]; then
  echo "Usage: $0 GROUP_NAME \"target1 target2 target3\" [BUILD_DIR] [CONFIG_STR]"
  exit 1
fi

# ANSI color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[0;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color
BOLD='\033[1m'

# Function to print colored text safely
print_colored() {
  local color=$1
  local text=$2
  echo -e "${color}${text}${NC}"
}

# Function to print progress without newline (for dynamic updates)
print_progress() {
  local color=$1
  local text=$2
  echo -ne "${color}${text}${NC}"
}

# Create log file with timestamp and configuration in the build directory
if [ -n "$CONFIG_STR" ]; then
  LOG_FILE="$BUILD_DIR/build_${GROUP_NAME}${CONFIG_STR}_$(date +%Y%m%d_%H%M%S).log"
else
  LOG_FILE="$BUILD_DIR/build_${GROUP_NAME}_$(date +%Y%m%d_%H%M%S).log"
fi

# Initialize counters
BUILT_COUNT=0
ERROR_COUNT=0
WARNING_COUNT=0
TOTAL_TARGETS=$(echo $TARGET_LIST | wc -w)

print_colored "$CYAN" ""
print_colored "$CYAN" "🔨 Building $GROUP_NAME benchmarks..."
print_colored "$CYAN" "📍 Build directory: $BUILD_DIR"
if [ -n "$CONFIG_STR" ]; then
  print_colored "$CYAN" "⚙️  Configuration: $CONFIG_STR"
fi
print_colored "$CYAN" "📝 Logging output to: $(basename $LOG_FILE)"
print_colored "$CYAN" "🎯 Targets: $TOTAL_TARGETS"
print_colored "$CYAN" ""

# Change to build directory
cd "$BUILD_DIR" || {
  print_colored "$RED" "❌ Error: Cannot change to build directory: $BUILD_DIR"
  exit 1
}

# Function to update progress counters from log file
update_counters() {
  BUILT_COUNT=$(grep "Built target bench-" "$LOG_FILE" 2>/dev/null | wc -l || echo 0)
  ERROR_COUNT=$(grep -i "error:" "$LOG_FILE" 2>/dev/null | wc -l || echo 0)
  WARNING_COUNT=$(grep -i "warning:" "$LOG_FILE" 2>/dev/null | wc -l || echo 0)
}

# Build targets and process output
make $TARGET_LIST 2>&1 | while IFS= read -r line; do
  # Log everything to file with line numbers for better tracking
  echo "$line" >> "$LOG_FILE"
  CURRENT_LINE=$(wc -l < "$LOG_FILE")
  
  # Track when a new target starts building (for warning counting)
  if echo "$line" | grep -q "Building.*target.*bench-"; then
    echo "TARGET_START:$CURRENT_LINE" >> "$LOG_FILE.markers"
  fi
  
  # Check for benchmark target completion (only count and show bench- targets)
  if echo "$line" | grep -q "Built target bench-"; then
    TARGET_NAME=$(echo "$line" | sed 's/.*Built target //' | sed 's/$//')
    # Update counters from file (since we're in a subshell) - only count bench- targets
    CURRENT_BUILT=$(grep "Built target bench-" "$LOG_FILE" | wc -l)
    PROGRESS=$(( (CURRENT_BUILT * 100) / TOTAL_TARGETS ))
    
    # Count warnings for this specific target since it started building
    if [ -f "$LOG_FILE.markers" ]; then
      LAST_START=$(tail -1 "$LOG_FILE.markers" 2>/dev/null | cut -d: -f2)
      if [ -n "$LAST_START" ]; then
        TARGET_WARNINGS=$(tail -n +$LAST_START "$LOG_FILE" | grep -i "warning:" | grep -v "LOAD segment with RWX permissions" | wc -l)
      else
        TARGET_WARNINGS=0
      fi
    else
      TARGET_WARNINGS=0
    fi
    
    if [ $TARGET_WARNINGS -gt 0 ]; then
      printf "${GREEN}✅ [%3d%%] Built: %-35s (%2d/%2d) ${YELLOW}⚠️ %d warnings${NC}\n" "$PROGRESS" "$TARGET_NAME" "$CURRENT_BUILT" "$TOTAL_TARGETS" "$TARGET_WARNINGS"
    else
      printf "${GREEN}✅ [%3d%%] Built: %-35s (%2d/%2d)${NC}\n" "$PROGRESS" "$TARGET_NAME" "$CURRENT_BUILT" "$TOTAL_TARGETS"
    fi
  fi
  
  # Check for errors
  if echo "$line" | grep -qi "error:"; then
    ERROR_MSG=$(echo "$line" | sed 's/.*error:/error:/')
    print_colored "$RED" "❌ Error: $ERROR_MSG"
  fi
  
  # Check for warnings (but not the LOAD segment RWX warnings which are normal for STM32)
  if echo "$line" | grep -qi "warning:" && ! echo "$line" | grep -q "LOAD segment with RWX permissions"; then
    CURRENT_WARNINGS=$(grep -i "warning:" "$LOG_FILE" | grep -v "LOAD segment with RWX permissions" | wc -l)
    # Only show first few warnings to avoid spam
    if [ $CURRENT_WARNINGS -le 3 ]; then
      WARNING_MSG=$(echo "$line" | sed 's/.*warning:/warning:/')
      print_colored "$YELLOW" "⚠️  Warning: $WARNING_MSG"
    fi
  fi
  
  # Show progress for percentage indicators
  if echo "$line" | grep -q "\[[0-9]*%\]"; then
    PERCENT=$(echo "$line" | grep -o "\[[0-9]*%\]" | head -1)
    print_progress "$BLUE" "\r🔄 Building... $PERCENT"
  fi
  
  # Show current target being compiled (only for benchmark targets)
  if echo "$line" | grep -q "Building.*object.*bench.*\.obj$"; then
    FILE_NAME=$(echo "$line" | sed 's/.*\/\([^/]*\)\.obj$/\1/')
    printf "\r🔧 Compiling: %-40s" "$FILE_NAME"
  fi
  
  # Show linking progress (only for benchmark targets)
  if echo "$line" | grep -q "Linking.*executable.*bench"; then
    TARGET_NAME=$(echo "$line" | sed 's/.*Linking.*executable[[:space:]]*\([^[:space:]]*\)$/\1/' | sed 's/.*\///')
    printf "\r🔗 Linking: %-40s" "$TARGET_NAME"
  fi
done

# Get the exit code from the make command
BUILD_EXIT_CODE=${PIPESTATUS[0]}

echo "" # New line after progress indicators

# Update final counters from log file
update_counters

# Filter out STM32-specific warnings that are normal
REAL_WARNING_COUNT=$(grep -i "warning:" "$LOG_FILE" 2>/dev/null | grep -v "LOAD segment with RWX permissions" | wc -l || echo 0)

# Final summary
print_colored "$BOLD$CYAN" ""
print_colored "$BOLD$CYAN" "=== BUILD SUMMARY FOR $GROUP_NAME ==="

if [ $BUILD_EXIT_CODE -eq 0 ] && [ $ERROR_COUNT -eq 0 ]; then
  print_colored "$GREEN" "✅ Successfully built: $BUILT_COUNT/$TOTAL_TARGETS targets"
else
  print_colored "$RED" "❌ Build failed: $ERROR_COUNT errors"
  if [ $BUILT_COUNT -gt 0 ]; then
    print_colored "$GREEN" "✅ Successful targets: $BUILT_COUNT/$TOTAL_TARGETS"
  fi
fi

if [ $REAL_WARNING_COUNT -gt 0 ]; then
  if [ $REAL_WARNING_COUNT -le 3 ]; then
    print_colored "$YELLOW" "⚠️  Warnings: $REAL_WARNING_COUNT"
  else
    print_colored "$YELLOW" "⚠️  Warnings: $REAL_WARNING_COUNT (showing first 3 above)"
  fi
fi

print_colored "$CYAN" "📝 Full log: $(basename $LOG_FILE)"
print_colored "$BOLD$CYAN" ""

# Clean up temporary markers file
rm -f "$LOG_FILE.markers"

# Return appropriate exit code
if [ $BUILD_EXIT_CODE -ne 0 ] || [ $ERROR_COUNT -gt 0 ]; then
  exit 1
else
  exit 0
fi 