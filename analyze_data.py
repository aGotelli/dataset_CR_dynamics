import csv
import numpy as np

# Load data
time_data, desired_data, actual_data, error_data, position_data = [], [], [], [], []

with open('data/position_tension_control_log.csv', 'r') as f:
    reader = csv.reader(f)
    next(reader)  # Skip header
    for row in reader:
        time_data.append(float(row[0]))
        desired_data.append(float(row[1]))
        actual_data.append(float(row[2]))
        error_data.append(float(row[3]))
        position_data.append(float(row[4]))

# Convert to numpy arrays
time_data = np.array(time_data)
desired_data = np.array(desired_data)
actual_data = np.array(actual_data)
error_data = np.array(error_data)
position_data = np.array(position_data)

print('=== DATA ANALYSIS ===')
print(f'Total duration: {time_data[-1]:.2f} seconds')
print(f'Data points: {len(time_data)}')
print(f'Average sampling rate: {len(time_data)/time_data[-1]:.1f} Hz')

print('\n=== DESIRED TENSION ===')
print(f'Range: {desired_data.min():.3f} to {desired_data.max():.3f} N')

print('\n=== ACTUAL TENSION ===')
print(f'Range: {actual_data.min():.3f} to {actual_data.max():.3f} N')
print(f'Mean: {actual_data.mean():.3f} N')
print(f'Std: {actual_data.std():.3f} N')

print('\n=== POSITION COMMAND ===')
print(f'Range: {position_data.min():.3f} to {position_data.max():.3f} rad')
print(f'Total position change: {position_data.max() - position_data.min():.3f} rad')
print(f'Position change rate: {(position_data.max() - position_data.min())/time_data[-1]:.3f} rad/s average')

print('\n=== ERROR ANALYSIS ===')
print(f'RMSE: {np.sqrt(np.mean(error_data**2)):.4f} N')
print(f'MAE: {np.mean(np.abs(error_data)):.4f} N')
print(f'Max Error: {np.max(np.abs(error_data)):.4f} N')

# Check for saturation or limits
print('\n=== POTENTIAL ISSUES ===')
if actual_data.max() < 0.5:
    print('⚠️  Low maximum tension reached - sensor may have issues or motor not moving enough')
    
position_derivative = np.diff(position_data)
max_position_rate = np.max(np.abs(position_derivative))
print(f'Max position change rate: {max_position_rate:.4f} rad/step')

# Check for step responses in position
large_steps = np.where(np.abs(position_derivative) > 0.01)[0]
if len(large_steps) > 0:
    print(f'Found {len(large_steps)} large position steps (>0.01 rad/step)')

# Check for motor limitations
print('\n=== MOTOR ANALYSIS ===')
print(f'Position command range: {position_data.min():.3f} to {position_data.max():.3f} rad')
print(f'Total motor movement: {abs(position_data.max() - position_data.min()):.3f} rad ({abs(position_data.max() - position_data.min()) * 180/np.pi:.1f} degrees)')

# Look for plateaus in actual tension (indicating motor limits)
tension_derivative = np.diff(actual_data)
small_changes = np.where(np.abs(tension_derivative) < 0.01)[0]
if len(small_changes) > len(actual_data) * 0.3:
    print('⚠️  Tension appears to plateau - motor may be hitting limits')

# Check if position commands are being followed
position_changes = np.abs(position_derivative)
avg_position_change = np.mean(position_changes)
print(f'Average position change per step: {avg_position_change:.4f} rad')
print(f'Max position change per step: {max_position_rate:.4f} rad')

# Tension tracking analysis
tracking_error = np.abs(error_data)
good_tracking = np.where(tracking_error < 0.1)[0]
print(f'\nTracking performance:')
print(f'- Points with error < 0.1N: {len(good_tracking)}/{len(error_data)} ({100*len(good_tracking)/len(error_data):.1f}%)')

# Check for control signal saturation
if np.std(position_data) < 0.01:
    print('⚠️  Very small position variations - control output may be limited')
