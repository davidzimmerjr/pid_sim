import time
import math
import matplotlib.pyplot as plt

m_car = 500  # kg
max_F = 8000  # N
min_F = -1000  # N
delay_factor = 0.5
cda = 0.4  # drag coefficient
rf = 0.05  # rolling resistance

Kp = 2
Ki = 1
max_error_int = 10
target_speed_base = 100  # km/h

dt = 0.1  # sec
sim_time = 60  # sec

# Changed to produce the sine wave graph from the article
target_speed_mode = 2


def main():
    error_int = 0
    state = {
        'time': 0,
        'speed': 0,
        'force': 0,
        'acc': 0,
        'force_drag': 0
    }

    # Lists to store data for plotting
    time_data = []
    speed_data = []
    target_speed_data = []

    while state['time'] < sim_time:
        target_speed = target_speed_base
        if target_speed_mode == 2:
            target_speed = target_speed_base * (1 + 0.2 * math.cos(state['time'] * 2 * 3.1415 / 20))
        if target_speed_mode == 3:
            target_speed = target_speed_base if int(state['time'] / 15) % 2 == 0 else target_speed_base * 0.8

        # controller
        error = target_speed - state['speed']
        error_int += error * dt
        error_int = max(min(error_int, max_error_int), -max_error_int)
        P_action = Kp * error
        I_action = Ki * error_int

        throttle = P_action + I_action
        target_force = throttle / 100 * max_F

        # car simulator
        state = sim_car(target_force, state)

        # Log data to lists instead of Marple
        time_data.append(state['time'])
        speed_data.append(state['speed'])
        target_speed_data.append(target_speed)

    # Plot the data using matplotlib
    plt.figure(figsize=(10, 6))
    plt.plot(time_data, speed_data, label='speed', color='blue')
    plt.plot(time_data, target_speed_data, '--', label='target_speed', color='green')
    plt.title('Sine target')
    plt.xlabel('Time (s)')
    plt.ylabel('Speed (km/h)')
    plt.legend()
    plt.grid(True)
    plt.show()


def sim_car(target_force, state):
    # add delay factor to simulate engine
    force = state['force'] * delay_factor + target_force * (1 - delay_factor)
    force = max(min(force, max_F), min_F)

    velocity = state['speed'] / 3.6

    # drag force
    force_drag = (0.5 * 1.225 * velocity * velocity * cda + rf * m_car * 9.81) * sign(velocity)

    acc = (force - force_drag) / m_car
    velocity += acc * dt

    state['time'] += dt
    state['force'] = force
    state['speed'] = velocity * 3.6
    state['acc'] = acc
    state['force_drag'] = force_drag

    return state


def sign(x):
    return 1 if x >= 0 else -1


if __name__ == '__main__':
    main()