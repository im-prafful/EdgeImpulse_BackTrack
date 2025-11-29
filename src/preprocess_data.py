import pandas as pd

df = pd.read_csv("data/raw/session.csv")
df = df.dropna()
df.to_csv("data/processed/imu_cleaned.csv", index=False)
print("Saved cleaned dataset.")
