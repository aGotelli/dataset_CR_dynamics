"""
ATI Force/Torque Sensor - Complete Data Collection and Analysis Pipeline
Combines simple_ati_ft.py data collection with interactive plotting and filtering
"""

import os
import sys
import time

def collect_and_plot_ati_data():
    """
    Complete workflow: collect ATI data and then plot it
    """
    print("=" * 70)
    print("ATI Force/Torque Sensor - Complete Data Collection & Analysis")
    print("=" * 70)
    
    # Import the modules
    try:
        from simple_ati_ft import SimpleATI_FT, quick_test, standard_acquisition
        from plot_6dof_interactive import plot_ati_data, plot_with_analysis
    except ImportError as e:
        print(f"Error importing modules: {e}")
        print("Make sure simple_ati_ft.py and plot_6dof_interactive.py are in the same directory")
        return
    
    # Menu for data collection
    print("\nStep 1: Data Collection")
    print("Choose data collection mode:")
    print("1. Quick test (5 seconds, 1000 Hz)")
    print("2. Standard acquisition (10 seconds, 1000 Hz)")
    print("3. High-speed acquisition (30 seconds, 2000 Hz)")
    print("4. Custom parameters")
    print("5. Skip collection (use existing data)")
    
    try:
        choice = input("\nEnter choice (1-5): ").strip()
        
        if choice == "1":
            print("\nCollecting quick test data...")
            success = quick_test()
            data_file = "quick_test.csv"
            
        elif choice == "2":
            print("\nCollecting standard data...")
            success = standard_acquisition()
            data_file = "ati_data.csv"
            
        elif choice == "3":
            print("\nCollecting high-speed data...")
            from simple_ati_ft import high_speed_acquisition
            success = high_speed_acquisition()
            data_file = "ati_highspeed.csv"
            
        elif choice == "4":
            # Custom parameters
            try:
                duration = int(input("Duration (seconds, default 10): ") or "10")
                rate = int(input("Sampling rate (Hz, default 1000): ") or "1000")
                output = input("Output filename (default ati_custom.csv): ") or "ati_custom.csv"
                
                print(f"\nCollecting custom data: {duration}s at {rate} Hz...")
                sensor = SimpleATI_FT(sampling_rate=rate)
                success = sensor.acquire_data(duration, output)
                data_file = output
                
            except ValueError:
                print("Invalid input, using standard acquisition")
                success = standard_acquisition()
                data_file = "ati_data.csv"
                
        elif choice == "5":
            print("\nSkipping data collection...")
            success = True
            data_file = None
            
        else:
            print("Invalid choice, using quick test")
            success = quick_test()
            data_file = "quick_test.csv"
        
        if choice != "5" and not success:
            print("Data collection failed. Check your DAQ connection.")
            return
        
        # Give a moment for file to be written
        if choice != "5":
            print("Data collection completed!")
            time.sleep(1)
        
        # Step 2: Data Analysis and Plotting
        print("\n" + "=" * 70)
        print("Step 2: Data Analysis and Plotting")
        print("=" * 70)
        
        print("Choose analysis mode:")
        print("1. Quick plot (basic filtering)")
        print("2. Interactive plot with filter controls")
        print("3. Frequency analysis + interactive plot")
        print("4. Auto-detect and plot best available data")
        
        plot_choice = input("\nEnter choice (1-4): ").strip()
        
        print("\nLaunching data plotter...")
        
        if plot_choice == "1":
            # Quick plot
            if data_file and os.path.exists(data_file):
                plot_ati_data(data_file, cutoff_freq=25)
            else:
                plot_ati_data()  # Auto-find data
                
        elif plot_choice == "2":
            # Interactive plot
            if data_file and os.path.exists(data_file):
                plot_ati_data(data_file)
            else:
                plot_ati_data()
                
        elif plot_choice == "3":
            # Frequency analysis + plot
            if data_file and os.path.exists(data_file):
                plot_ati_data(data_file, show_freq_analysis=True)
            else:
                plot_with_analysis()
                
        else:
            # Auto-detect
            plot_ati_data()
        
        print("\nAnalysis completed!")
        
    except KeyboardInterrupt:
        print("\nOperation cancelled by user")
    except Exception as e:
        print(f"Error: {e}")

def quick_demo():
    """
    Quick demonstration - collect 5 seconds of data and plot it
    """
    print("=" * 50)
    print("Quick ATI F/T Demo")
    print("=" * 50)
    
    try:
        from simple_ati_ft import quick_test
        from plot_6dof_interactive import plot_ati_data
        
        print("Collecting 5 seconds of test data...")
        success = quick_test()
        
        if success:
            print("Data collected! Launching plotter...")
            time.sleep(1)
            plot_ati_data("quick_test.csv", cutoff_freq=20)
        else:
            print("Data collection failed")
            
    except Exception as e:
        print(f"Error in demo: {e}")

def main_menu():
    """
    Main menu for the complete ATI F/T workflow
    """
    print("=" * 70)
    print("ATI Force/Torque Sensor - Complete Workflow")
    print("=" * 70)
    print("Choose an option:")
    print("1. Complete workflow (collect + analyze)")
    print("2. Quick demo (5 sec collection + plot)")
    print("3. Data collection only")
    print("4. Data analysis only (existing files)")
    print("=" * 70)
    
    try:
        choice = input("Enter choice (1-4): ").strip()
        
        if choice == "1":
            collect_and_plot_ati_data()
        elif choice == "2":
            quick_demo()
        elif choice == "3":
            from simple_ati_ft import main as ati_main
            ati_main()
        elif choice == "4":
            from plot_6dof_interactive import plot_ati_data
            plot_ati_data()
        else:
            print("Invalid choice, running complete workflow")
            collect_and_plot_ati_data()
            
    except KeyboardInterrupt:
        print("\nGoodbye!")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    if len(sys.argv) > 1:
        command = sys.argv[1].lower()
        if command == "demo":
            quick_demo()
        elif command == "collect":
            from simple_ati_ft import main as ati_main
            ati_main()
        elif command == "plot":
            from plot_6dof_interactive import plot_ati_data
            plot_ati_data()
        else:
            print("Unknown command. Available: demo, collect, plot")
    else:
        main_menu()
