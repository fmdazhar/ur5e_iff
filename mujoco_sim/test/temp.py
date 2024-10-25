import time
import numpy as np
import matplotlib.pyplot as plt
import multiprocessing

# Function to generate random data and put it in the queue
def generate_data(queue):
    step_count = 0
    while True:
        data = {
            'time_step': step_count,
            'value': np.sin(step_count * 0.1) + np.random.normal(0, 0.1)
        }
        queue.put(data)
        step_count += 1
        print(step_count)
        time.sleep(0.1)

# Function to plot data from the queue
def plot_data(queue):
    plt.ion()  # Turn on interactive mode for live plotting
    fig, ax = plt.subplots()
    ax.set_title("Live Data Plotting")
    ax.set_xlabel("Time Step")
    ax.set_ylabel("Value")

    time_steps = []
    values = []

    while True:
        if not queue.empty():
            data = queue.get()
            time_steps.append(data['time_step'])
            values.append(data['value'])

            # Limit the data lists to the last 50 points
            max_length = 50
            if len(time_steps) > max_length:
                time_steps = time_steps[-max_length:]
                values = values[-max_length:]

            ax.clear()
            ax.plot(time_steps, values, label='Value', color='b')
            ax.set_title("Live Data Plotting")
            ax.set_xlabel("Time Step")
            ax.set_ylabel("Value")
            ax.legend()

            plt.pause(0.001)

# Setup multiprocessing
if __name__ == "__main__":
    try:
        # Create a multiprocessing queue
        data_queue = multiprocessing.Queue()

        # Start the plotting process
        plot_process = multiprocessing.Process(target=plot_data, args=(data_queue,), daemon=True)
        plot_process.start()

        # Run the data generation in the main process
        generate_data(data_queue)
    except KeyboardInterrupt:
        print("Process interrupted by user.")
