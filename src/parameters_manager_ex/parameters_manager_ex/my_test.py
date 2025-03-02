import yaml


YAML_DATA = """
    preset: "low"
    width: 640
    height: 480
    high:
        fps: 5
        bitrate: 200
        iframe_interval: 5
        vbv: 200
    low:
        fps: 5
        bitrate: 200
        iframe_interval: 5
        vbv: 200
        """

data = yaml.safe_load(YAML_DATA)

keys = ["preset", "high.fps"]

for key in keys:
    source = data
    items = key.split(".")
    my_key = items[-1]
    items = items[:-1]

    for item in items:
        source = source[item]

    source[my_key] = 10

print(data)