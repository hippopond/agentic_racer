import json
import os
import sys

def main():
    print("🤖 Agentic Reviewer Booting Up...")
    
    # 1. Read the physical test results
    report_path = 'test_report.json'
    if not os.path.exists(report_path):
        print("❌ No test report found! Run the simulation eval first.")
        sys.exit(1)
        
    with open(report_path, 'r') as f:
        test_results = json.load(f)
        
    print(f"📊 Physical Eval Results: {test_results}")
    
    # 2. Extract git diff (Mocked for now)
    git_diff = "+\tlookahead_distance = 2.5\n-\tlookahead_distance = 1.0"
    
    # 3. Simulate calling an LLM API (CodeRabbit workflow)
    prompt = f"""
    You are an automated code reviewer. 
    The user changed the following code:
    {git_diff}
    
    The physical test results in the simulator were:
    {test_results}
    
    Write a PR review comment.
    """
    
    print("\n--- Generating PR Review ---")
    print(f"Executing Prompt: {prompt}")
    print("\n[AI Review]: I see you increased the lookahead_distance. While this is mathematically faster, the car crashed into the wall because the lookahead exceeded the track width. Rejecting PR. Please tune down to 1.5.")

if __name__ == '__main__':
    main()
