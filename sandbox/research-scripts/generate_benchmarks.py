#!/usr/bin/env python3
# Generate remaining benchmark files
import os

# Define the variants
algorithms = ['mahony', 'madgwick']
scalars = [
    ('double', 'double', ''),
    ('q7_24', 'Q7_24', 'Fixed'),
    ('q3_12', 'Q3_12', 'Fixed')
]
modes = [('imu', 'false'), ('marg', 'true')]

# Template for benchmark files
template = '''#include <ento-bench/harness.h>
#include <ento-util/file_path_util.h>
#include <ento-util/debug.h>
#include <ento-util/unittest.h>

#include <ento-mcu/cache_util.h>
#include <ento-mcu/flash_util.h>
#include <ento-mcu/clk_util.h>
#include <ento-state-est/attitude-est/attitude_estimation_problem.h>
#include <ento-state-est/attitude-est/{algorithm}{fixed_suffix}.h>

extern "C" void initialise_monitor_handles(void);

using namespace EntoBench;
using namespace EntoUtil;
using namespace EntoAttitude;

int main()
{{
  using Scalar = {scalar_type};
  using Filter = Filter{algorithm_cap}{fixed_suffix}<Scalar, {use_mag}>; // {mode_desc}
  using Problem = AttitudeProblem<Scalar, Filter, {use_mag}>;
  
  initialise_monitor_handles();

  // Configure max clock rate and set flash latency
  sys_clk_cfg();

  // Turn on caches if applicable
  enable_instruction_cache();
  enable_instruction_cache_prefetch();
  icache_enable();

  const char* base_path = DATASET_PATH;
  const char* rel_path = "state-est/benchmark_{dataset}_dataset.txt";
  char dataset_path[512];
  char output_path[256];

  if (!EntoUtil::build_file_path(base_path, rel_path,
                                 dataset_path, sizeof(dataset_path)))
  {{
    ENTO_DEBUG("ERROR! Could not build file path for bench_{filename}!");
  }}

  // Create filter with typical gains{gains_comment}
  Filter filter{gains};
  Problem problem(filter);

  printf("File path: %s", dataset_path);
  EntoBench::Harness harness(problem, "Bench {display_name}",
                             dataset_path,
                             output_path);

  harness.run();

  exit(1);
  return 0;
}}'''

# Generate files
for algorithm in algorithms:
    for scalar_name, scalar_type, fixed_suffix in scalars:
        for mode_name, use_mag in modes:
            # Skip files that already exist
            filename = f'bench_{algorithm}_{scalar_name}_{mode_name}.cc'
            filepath = f'benchmark/state-est/attitude-est/{filename}'
            
            if os.path.exists(filepath):
                print(f'Skipping {filename} (already exists)')
                continue
                
            # Determine parameters
            algorithm_cap = algorithm.capitalize()
            mode_desc = 'MARG (with magnetometer)' if use_mag == 'true' else 'IMU only (no magnetometer)'
            dataset = 'marg' if use_mag == 'true' else 'imu'
            display_name = f'{algorithm_cap} {scalar_type} {mode_name.upper()}'
            
            # Set gains based on algorithm and scalar type
            if algorithm == 'mahony':
                if fixed_suffix:
                    gains = '(Scalar(1.0f), Scalar(0.1f))'
                    gains_comment = ' (converted to fixed-point)'
                else:
                    gains = ''
                    gains_comment = ''
            else:  # madgwick
                if fixed_suffix:
                    gains = '(Scalar(0.1f))'
                    gains_comment = ' (converted to fixed-point)'
                else:
                    gains = ''
                    gains_comment = ''
            
            # Generate file content
            content = template.format(
                algorithm=algorithm,
                algorithm_cap=algorithm_cap,
                fixed_suffix='_fixed' if fixed_suffix else '',
                scalar_type=scalar_type,
                use_mag=use_mag,
                mode_desc=mode_desc,
                dataset=dataset,
                filename=filename.replace('.cc', ''),
                display_name=display_name,
                gains=gains,
                gains_comment=gains_comment
            )
            
            # Write file
            with open(filepath, 'w') as f:
                f.write(content)
            print(f'Generated {filename}')

print('Done generating benchmark files!') 