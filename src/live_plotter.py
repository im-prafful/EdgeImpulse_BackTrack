import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("data/raw/session.csv")
df[["ax", "ay", "az"]].plot()
plt.title("Accelerometer")
plt.show()
