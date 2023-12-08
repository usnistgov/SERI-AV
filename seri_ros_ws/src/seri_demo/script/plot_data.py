import pandas as pd
import matplotlib.pyplot as plt

# Read CSV files into Pandas DataFrames
df1 = pd.read_csv('/home/zeid/seri_nist_ws/seri_ros_ws/src/seri_demo/script/data/distance_no_delay.csv')
# df2 = pd.read_csv('/home/zeid/seri_nist_ws/seri_ros_ws/src/seri_demo/script/data/distance_200_delay.csv')

# Plot data from DataFrames using Matplotlib
plt.plot(df1['delay'], df1['distance'], label='No additional delay')
# plt.plot(df2['__time'], df2['/collect_data/distance'], label='200 ms delay')

plt.xlabel('ROS clock')
plt.ylabel('Distance ego-front (m)')
plt.legend()
plt.show()
