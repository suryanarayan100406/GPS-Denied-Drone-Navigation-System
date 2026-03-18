#!/usr/bin/env python3
"""
Robothon Testing Environment Runner and Evaluator
Automates testing across multiple difficulty levels and generates evaluation reports
"""

import subprocess
import json
import time
import sys
import os
from pathlib import Path
from datetime import datetime


class RobothonTester:
    """Automated testing framework for Robothon evaluation."""
    
    def __init__(self, workspace_path: str = '/mnt/c/Users/samai/Desktop/drone'):
        self.workspace = workspace_path
        self.results_dir = Path(f'{workspace_path}/results')
        self.results_dir.mkdir(exist_ok=True)
        self.wsl_distro = 'Ubuntu'
        
        # Test configurations
        self.test_configs = {
            'easy': {
                'difficulty': 'easy',
                'description': 'Simple open environment, minimal obstacles',
                'duration': 30,
                'enable_dynamic': False,
                'expected_completion': True,
                'expected_safety_score': '>= 95'
            },
            'medium': {
                'difficulty': 'medium',
                'description': 'Moderate complexity with moving obstacles',
                'duration': 60,
                'enable_dynamic': True,
                'expected_completion': True,
                'expected_safety_score': '>= 80'
            },
            'hard': {
                'difficulty': 'hard',
                'description': 'Dense obstacles, dynamic elements, harsh conditions',
                'duration': 90,
                'enable_dynamic': True,
                'expected_completion': True,
                'expected_safety_score': '>= 70'
            }
        }
    
    def run_test(self, difficulty: str, test_id: str = None) -> dict:
        """Run a single test configuration."""
        if difficulty not in self.test_configs:
            print(f"❌ Unknown difficulty level: {difficulty}")
            return {}
        
        config = self.test_configs[difficulty]
        test_id = test_id or f"{difficulty}_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        
        print(f"\n{'='*70}")
        print(f"🚀 Starting Test: {difficulty.upper()}")
        print(f"{'='*70}")
        print(f"Description: {config['description']}")
        print(f"Duration: {config['duration']}s")
        print(f"Dynamic Obstacles: {config['enable_dynamic']}")
        
        # Build command
        cmd = [
            'wsl', '-d', self.wsl_distro, '--', 'bash', '-lc',
            f"""
            export WEBOTS_HOME=$HOME/webots;
            source /opt/ros/jazzy/setup.bash;
            source {self.workspace}/install/setup.bash;
            cd {self.workspace};
            
            timeout {config['duration']} ros2 launch drone_nav_2d drone_nav_headless.py \
              difficulty_level:={difficulty} \
              enable_dynamic_obstacles:={str(config['enable_dynamic']).lower()} \
              bag_output:=bags/{test_id} \
              2>&1
            """
        ]
        
        print(f"\n▶️  Running test (this may take a few minutes)...")
        start_time = time.time()
        
        try:
            result = subprocess.run(
                cmd,
                capture_output=True,
                text=True,
                timeout=config['duration'] + 30
            )
            
            elapsed = time.time() - start_time
            
            # Parse results
            test_result = {
                'test_id': test_id,
                'difficulty': difficulty,
                'status': 'completed' if result.returncode == 0 else 'timeout_or_error',
                'elapsed_seconds': elapsed,
                'timestamp': datetime.now().isoformat(),
                'stdout': result.stdout[-1000:] if result.stdout else '',  # Last 1000 chars
                'config': config
            }
            
            if result.returncode == 0:
                print(f"✅ Test completed successfully in {elapsed:.1f}s")
            else:
                print(f"⏱️  Test timed out or encountered error after {elapsed:.1f}s")
            
            return test_result
            
        except subprocess.TimeoutExpired:
            print(f"⏱️  Test timeout after {config['duration'] + 30}s")
            return {
                'test_id': test_id,
                'difficulty': difficulty,
                'status': 'timeout',
                'config': config,
                'timestamp': datetime.now().isoformat()
            }
        except Exception as e:
            print(f"❌ Test failed with error: {e}")
            return {
                'test_id': test_id,
                'difficulty': difficulty,
                'status': 'error',
                'error': str(e),
                'config': config,
                'timestamp': datetime.now().isoformat()
            }
    
    def run_all_tests(self) -> dict:
        """Run all test configurations."""
        print("\n" + "="*70)
        print("🤖 ROBOTHON DRONE NAVIGATION EVALUATION SUITE")
        print("="*70)
        print(f"Workspace: {self.workspace}")
        print(f"Results will be saved to: {self.results_dir}")
        
        all_results = {
            'suite_start': datetime.now().isoformat(),
            'tests': {},
            'summary': {}
        }
        
        for difficulty in ['easy', 'medium', 'hard']:
            result = self.run_test(difficulty)
            all_results['tests'][difficulty] = result
        
        all_results['suite_end'] = datetime.now().isoformat()
        
        # Generate summary
        all_results['summary'] = self.generate_summary(all_results['tests'])
        
        # Save results
        self.save_results(all_results)
        
        # Print summary
        self.print_summary(all_results['summary'])
        
        return all_results
    
    def generate_summary(self, test_results: dict) -> dict:
        """Generate summary of all test results."""
        summary = {
            'total_tests': len(test_results),
            'completed': sum(1 for r in test_results.values() if r.get('status') == 'completed'),
            'failed': sum(1 for r in test_results.values() if r.get('status') in ['error', 'timeout']),
            'completion_rate': 0
        }
        
        if summary['total_tests'] > 0:
            summary['completion_rate'] = (summary['completed'] / summary['total_tests']) * 100
        
        return summary
    
    def save_results(self, results: dict):
        """Save results to JSON file."""
        filename = self.results_dir / f"evaluation_suite_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        with open(filename, 'w') as f:
            json.dump(results, f, indent=2)
        print(f"\n✅ Results saved to: {filename}")
    
    def print_summary(self, summary: dict):
        """Print test summary."""
        print(f"\n{'='*70}")
        print("📊 TEST SUMMARY")
        print(f"{'='*70}")
        print(f"Total Tests: {summary['total_tests']}")
        print(f"Completed: {summary['completed']}")
        print(f"Failed: {summary['failed']}")
        print(f"Completion Rate: {summary['completion_rate']:.1f}%")
        print(f"{'='*70}\n")


def main():
    """Run the evaluation suite."""
    tester = RobothonTester()
    
    print("\nSelect test option:")
    print("1. Run all tests (easy -> medium -> hard)")
    print("2. Run easy test only")
    print("3. Run medium test only")
    print("4. Run hard test only")
    print("5. Generate documentation")
    
    choice = input("\nEnter choice (1-5): ").strip()
    
    if choice == '1':
        tester.run_all_tests()
    elif choice == '2':
        tester.run_test('easy')
    elif choice == '3':
        tester.run_test('medium')
    elif choice == '4':
        tester.run_test('hard')
    elif choice == '5':
        generate_environment_documentation(tester)
    else:
        print("Invalid choice")


def generate_environment_documentation(tester: RobothonTester):
    """Generate environment documentation."""
    doc = """
# Testing Environment Documentation

## Environment Specifications

### Easy Level
- **Obstacle Density**: 15%
- **Obstacles**: Static, no movement
- **Terrain**: Flat, smooth
- **Sensor Noise**: Low (std=0.01)
- **Processing Delay**: 10ms
- **Expected Behavior**: Fast convergence, minimal replanning
- **Safety Requirement**: >= 95 score

### Medium Level
- **Obstacle Density**: 25%
- **Obstacles**: 2 dynamic obstacles moving unpredictably
- **Terrain**: Rough, variable (roughness=0.3)
- **Wind Effect**: 2 m/s simulated wind
- **Sensor Noise**: Moderate (std=0.05)
- **Processing Delay**: 30ms
- **Expected Behavior**: Adaptive planning, moderate replanning
- **Safety Requirement**: >= 80 score

### Hard Level
- **Obstacle Density**: 40%
- **Obstacles**: 5 dynamic obstacles with rapid movement
- **Terrain**: Very rough (roughness=0.7)
- **Wind Effect**: 5 m/s simulated wind
- **Sensor Noise**: High (std=0.1)
- **Processing Delay**: 50ms
- **Expected Behavior**: Aggressive replanning, dynamic adaptation required
- **Safety Requirement**: >= 70 score

## Performance Metrics

1. **Navigation Metrics**
   - Path optimality (straight-line distance / actual distance)
   - Replan count (lower is better)
   - Initial plan quality

2. **Safety Metrics**
   - Minimum obstacle distance
   - Collision events
   - Safety score (100 - 10*collisions)

3. **Energy Metrics**
   - Battery efficiency
   - Energy consumption rate
    - Energy per distance traveled

4. **Computational Metrics**
   - Average CPU usage
   - Peak memory usage
   - Command frequency

5. **Overall Score**
   - Mission completion (50 points)
   - Safety (20 points)
   - Energy efficiency (20 points)
   - Path optimality (10 points)
"""
    
    doc_path = tester.results_dir / "ENVIRONMENT_DOCUMENTATION.md"
    with open(doc_path, 'w') as f:
        f.write(doc)
    
    print(f"\n✅ Documentation saved to: {doc_path}")


if __name__ == '__main__':
    # For WSL/Linux
    if len(sys.argv) > 1:
        difficulty = sys.argv[1]
        tester = RobothonTester()
        result = tester.run_test(difficulty)
    else:
        main()
