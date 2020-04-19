import pandas as pd
import random

# Generate CPP-ish data for network to be copied to airports.h.
# Sample a bunch of cities from uscities.csv and rewrite them in the desired format to
# city_data.cxx. That data can then be copy-pasted to airports.h. A little awkward, but it works.

df = pd.read_csv('uscities.csv')
num_cities = len(df.index)

# Randomly select N cities
N = 1000

min_charge_rate = 70
max_charge_rate = 140


def TransformRow(row: pd.Series):
  name = row['city_ascii'].replace(" ", "_").replace("'", "")
  name = name + "_" + row['state_id']

  charge_rate = random.uniform(min_charge_rate, max_charge_rate)
  return pd.Series([name, row['lat'], row['lng'], charge_rate],
                   index=['airport_name', 'latitude', 'longitude', 'charge_rate'])


df_sample = df.sample(n=N).apply(TransformRow, axis=1, raw=False)
with open('city_data.cxx', 'w') as f:
  f.write("{{")
  for index, row in df_sample.iterrows():
    f.write("{")
    f.write('"')
    f.write(row['airport_name'])
    f.write('", ')
    f.write(str(row['latitude']))
    f.write(', ')
    f.write(str(row['longitude']))
    f.write(', ')
    f.write(str(row['charge_rate']))
    f.write("},\n")
  f.write("}};")
