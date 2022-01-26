"""
Parse convert bag files to csv files.
"""

import rosbag2_py as bag
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from topic_writer.csv_writer import CsvHelper


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = bag.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = bag.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )
    return storage_options, converter_options


def write_bag_to_csv(bag_path, csv_path, verbose=True):
    csv_writer = CsvHelper()

    reader = bag.SequentialReader()
    storage_options, converter_options = get_rosbag_options(bag_path)
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()

    type_map = {
        topic_types[i].name: topic_types[i].type for i in range(len(topic_types))
    }

    topics = set()
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        if topic[0] == "/":
            topic = topic[1:]

        csv_writer.callback_message(msg, topic)
        if verbose and not topic in topics:
            topics.add(topic)

    if verbose:
        print("saw topics:", topics)
        print("header:", csv_writer.header)
        print("number of rows:", len(csv_writer.rows))

    success = csv_writer.write_file(csv_path)
    if success:
        print(f"Wrote file {csv_path}")
    else:
        print(f"Could not save {csv_path}")


if __name__ == "__main__":
    import os
    from utils.custom_argparser import exp_parser, check_platform

    parser = exp_parser(description=__doc__)
    args = parser.parse_args()

    check_platform(args)

    for exp_name in args.experiment_names:
        print(f"Treating experiment {exp_name}")
        dirname = f"{args.experiment_root}/{exp_name}/"
        filenames = os.listdir(dirname)
        for filename in filenames:

            # identify bag file folders
            if not os.path.exists(f"{dirname}/{filename}/metadata.yaml"):
                continue
            bag_path = f"{dirname}/{filename}/"
            csv_path = f"{dirname}/csv_files/{filename}.csv"
            write_bag_to_csv(bag_path, csv_path)
