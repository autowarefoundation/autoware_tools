#! /usr/bin/python3
import math

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy
from rclpy.node import Node
from sklearn.metrics import mean_absolute_error
from sklearn.metrics import mean_squared_error
from sklearn.metrics import r2_score
from sklearn.model_selection import train_test_split
import torch
import torch.nn as nn
import torch.optim as optim


class NeuralNetworkThrottle(Node):
    class NeuralNetwork(nn.Module):
        def __init__(self):
            super(NeuralNetworkThrottle.NeuralNetwork, self).__init__()
            self.fc1 = nn.Linear(2, 128)  # Input layer with 2 neurons, hidden layer with n neurons
            self.relu1 = nn.ReLU()
            self.fc2 = nn.Linear(128, 32)
            self.relu2 = nn.ReLU()
            self.fc3 = nn.Linear(32, 1)  # Output layer with 1 neuron

        def forward(self, x):
            x = self.fc1(x)
            x = self.relu1(x)
            x = self.fc2(x)
            x = self.relu2(x)
            x = self.fc3(x)

            return x

    def __init__(self):

        super().__init__("neural_network_throttle")

        self.model = self.NeuralNetwork()

        data = pd.read_csv("throttling.csv")
        dataa = pd.read_csv("throttling.csv")
        ush = pd.read_csv("throttling.csv")

        # Declare params from launch file
        self.declare_parameter("filter_vel_throttle", 10.0)
        self.declare_parameter("filter_cmd_throttle", 10.0)
        self.declare_parameter("filter_acc_throttle", 10.0)

        # Load params from launch file
        self.FILTER_VEL_THROTTLE = (
            self.get_parameter("filter_vel_throttle").get_parameter_value().double_value
        )
        self.FILTER_CMD_THROTTLE = (
            self.get_parameter("filter_cmd_throttle").get_parameter_value().double_value
        )
        self.FILTER_ACC_THROTTLE = (
            self.get_parameter("filter_acc_throttle").get_parameter_value().double_value
        )

        mean0 = data["Velocity"].mean()
        std0 = data["Velocity"].std()
        data["Velocity"] = (data["Velocity"] - mean0) / std0
        dataa["Velocity"] = (dataa["Velocity"] - mean0) / std0

        data = data[abs(data["Velocity"] - mean0) <= std0 * self.FILTER_VEL_THROTTLE]
        dataa = dataa[abs(dataa["Velocity"] - mean0) <= std0 * self.FILTER_VEL_THROTTLE]

        mean1 = data["Throttling"].mean()
        std1 = data["Throttling"].std()
        data["Throttling"] = (data["Throttling"] - mean1) / std1
        dataa["Throttling"] = (dataa["Throttling"] - mean1) / std1

        data = data[abs(data["Throttling"] - mean1) <= std1 * self.FILTER_CMD_THROTTLE]
        dataa = dataa[abs(dataa["Throttling"] - mean1) <= std1 * self.FILTER_CMD_THROTTLE]

        mean2 = data["Acceleration_measured"].mean()
        std2 = data["Acceleration_measured"].std()
        data["Acceleration_measured"] = (data["Acceleration_measured"] - mean2) / std2
        dataa["Acceleration_measured"] = (dataa["Acceleration_measured"] - mean2) / std2

        data = data[abs(data["Acceleration_measured"] - mean2) <= std2 * self.FILTER_ACC_THROTTLE]
        dataa = dataa[
            abs(dataa["Acceleration_measured"] - mean2) <= std2 * self.FILTER_ACC_THROTTLE
        ]

        # Split the data into input features (velocity and throttle) and target (acceleration) and test/train

        X = data[["Velocity", "Throttling"]].values
        y = data["Acceleration_measured"].values

        X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

        # Convert NumPy arrays to PyTorch tensors
        X_train = torch.tensor(X_train, dtype=torch.float32)
        y_train = torch.tensor(y_train, dtype=torch.float32)
        X_test = torch.tensor(X_test, dtype=torch.float32)
        y_test = torch.tensor(y_test, dtype=torch.float32)

        criterion = nn.MSELoss()
        optimizer = optim.Adam(self.model.parameters(), lr=0.001)  # , weight_decay=0.001)

        # Training loop
        num_epochs = 100
        for epoch in range(num_epochs):
            # Forward pass
            outputs = self.model(X_train)

            loss = criterion(outputs, y_train.view(-1, 1))

            # Backpropagation and optimization
            optimizer.zero_grad()
            loss.backward()
            optimizer.step()

        with torch.no_grad():
            test_outputs = self.model(X_test)

        # Visualization

        velocity_range = np.linspace(0, (X[:, 0] * std0 + mean0).max(), 20)
        # throttling_range = np.linspace((X[:, 1]*std1+mean1).min(), (X[:, 1]*std1+mean1).max(), 20)
        throttling_range = np.linspace(0, (X[:, 1] * std1 + mean1).max(), 20)
        V, A = np.meshgrid(velocity_range, throttling_range)

        input_grid = np.column_stack(((V.flatten() - mean0) / std0, (A.flatten() - mean1) / std1))
        input_grid = torch.tensor(input_grid, dtype=torch.float32)

        with torch.no_grad():
            commands = self.model(input_grid).reshape(V.shape)

        commands_new = commands * std2 + mean2

        # Save the trained model
        # torch.save(self.model.state_dict(), 'trained_throttle.pth')

        # evaluation
        mse = mean_squared_error(y_test, test_outputs.view(-1).numpy())
        self.get_logger().info(f"Mean Squared Error on Test Data: {mse}")

        mae = mean_absolute_error(y_test, test_outputs.view(-1).numpy())
        self.get_logger().info(f"Mean Absolute Error on Test Data: {mae}")

        rmse = math.sqrt(mse)
        self.get_logger().info(f"Root Mean Squared Error on Test Data: {rmse}")

        r2 = r2_score(y_test, test_outputs.view(-1).numpy())
        self.get_logger().info(f"R-squared (R2) Score on Test Data: {r2}")

        # Save NN model in csv correct format for testing in the real vehicle

        velocity_headers = ["{:.2f}".format(v) for v in velocity_range]

        # we normalize throttling values between 0 and 1
        throttling_range /= 100
        # throttling_headers = ['Throttling {:.2f}'.format(a) for a in throttling_range]

        headers = [""] + velocity_headers

        commands_new_with_throttling = np.column_stack((throttling_range, commands_new))

        csv_filename = "accel_map.csv"
        np.savetxt(
            csv_filename,
            commands_new_with_throttling,
            delimiter=",",
            header=",".join(headers),
            comments="",
        )

        # 3D Visualization (plot)
        xdata = dataa.Velocity * std0 + mean0
        ydata = dataa.Throttling * std1 + mean1
        zdata = dataa.Acceleration_measured * std2 + mean2

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        ax.scatter3D(xdata, ydata, zdata, c=zdata, marker="o")
        surf = ax.plot_surface(V, A, commands_new, cmap="viridis")

        ax.set_xlabel("Velocity")
        ax.set_zlabel("Acceleration")
        ax.set_ylabel("Throttling Output")
        ax.set_title("Neural Network Output vs. Velocity and Throttling")

        plt.figure(figsize=(10, 6))
        plt.subplot(3, 1, 1)
        plt.hist(ush["Velocity"], bins=20, color="skyblue", edgecolor="black")
        plt.title("Distribution of Velocity")
        plt.xlabel("Velocity")
        plt.ylabel("Frequency")

        # Plot the distribution of 'Throttling'
        plt.subplot(3, 1, 2)
        plt.hist(ush["Throttling"], bins=20, color="salmon", edgecolor="black")
        plt.title("Distribution of Throttling")
        plt.xlabel("Throttling")
        plt.ylabel("Frequency")

        # Plot the distribution of 'Acceleration_measured'
        plt.subplot(3, 1, 3)
        plt.hist(ush["Acceleration_measured"], bins=20, color="lightgreen", edgecolor="black")
        plt.title("Distribution of Acceleration")
        plt.xlabel("Acceleration")
        plt.ylabel("Frequency")

        plt.tight_layout()

        fig.colorbar(surf)

        plt.show()


def main():
    rclpy.init()
    neural_network_throttle = NeuralNetworkThrottle()
    rclpy.spin(neural_network_throttle)


if __name__ == "__main__":
    main()
