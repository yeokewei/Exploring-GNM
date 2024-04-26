import os
import argparse
import numpy as np

import rosbag
from geometry_msgs.msg import Pose
# from nav_msgs.msg import Odometry

NAV_ROSBAG_DIR = "../nav_rosbag"
TOPO_ROSBAG_DIR = "../topomaps/bags"
TOPIC = "/odom"

def calculate_distance(pose1, pose2):
    """Calculate Euclidean distance between two poses."""
    return np.sqrt((pose1.x - pose2.x)**2 + (pose1.y - pose2.y)**2 + (pose1.z - pose2.z)**2)

def extract_positions(bagfile, topic):
    """Extracts positions from a ROS bag file."""
    bag = rosbag.Bag(bagfile)
    positions = []
    for topic, msg, t in bag.read_messages(topics=[topic]):
        positions.append(msg.pose.pose.position)
    bag.close()
    return positions

def compute_total_distance(positions):
    """Compute the total distance traveled between all consecutive positions."""
    total_distance = 0
    for i in range(1, len(positions)):
        total_distance += calculate_distance(positions[i-1], positions[i])
    return total_distance

def main(args: argparse.Namespace):
    """
    Main loop to calculate performance of topological and navigation bags.
    Optional: Get the topological and navigation bag file paths from args.
    Returns the mean distance between the last positions of the topological and navigation bags.
    """
    distances_arr = [] # to store distances for each nav_odom bag
    topo_name_rosbag_dir = os.path.join(TOPO_ROSBAG_DIR, args.bag+'.bag')
    
    # Get total distance travelled in topological bag
    goal_positions = extract_positions(topo_name_rosbag_dir, TOPIC)
    total_dist_travelled = compute_total_distance(goal_positions)
    print(f"Total distance travelled in {args.bag}: {total_dist_travelled} m")

    goal_position = goal_positions[-1]
    for i in range(args.count): # loop through nav_odom bags
        
        # get the topological and navigation bag file paths
        
        nav_name_rosbag_dir = os.path.join(NAV_ROSBAG_DIR, args.bag+'_'+str(i+1)+'.bag')
        # topo_name_rosbag_dir = os.path.join(TOPO_ROSBAG_DIR, args.topo)
        # nav_name_rosbag_dir = os.path.join(NAV_ROSBAG_DIR, args.nav)
        
        # extract last positions from topological and navigation bags
        
        test_goal_positions = extract_positions(nav_name_rosbag_dir, TOPIC)
        
        # calculate distance between last positions
        distance = calculate_distance(goal_position, test_goal_positions[-1]) 
        
        distances_arr.append(distance)
        # success_rate = np.mean(progress_scores)
        print(f"Attempt {i+1}: {distance:.1f} m (from Goal) {100-((distance/total_dist_travelled)*100):.1f} % Completed")
        
    print(f"Mean distance from Goal for {args.bag}: {np.mean(distances_arr):.1f} m")
    print(f"Percentage Completed {args.bag}: {100 - ((np.mean(distances_arr)/total_dist_travelled)*100):.1f} %")

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(
        description=f"Code to compute performance {TOPIC} topic in topomaps & nav_rosbag"
    )
    parser.add_argument(
        "--bag",
        "-b",
        default="cafe1",
        type=str,
        help="Assumes naming for topobag is X.bag and navbag is X_1.bag (default: cafe1)",
    )
    
    parser.add_argument(
        "--count",
        "-c",
        default=5,
        type=int,
        help="Set the number of nav_odom to test (default: 5)",
    )
    
    # parser.add_argument(
    #     "--topo",
    #     "-t",
    #     default="cafe1.bag",
    #     type=str,
    #     help="path to topological step odom bag in {TOPO_ROSBAG_DIR} directory (default: test4.bag)",
    # )
    # parser.add_argument(
    #     "--nav",
    #     "-n",
    #     default="cafe1_1.bag",
    #     type=str,
    #     help=f"path to navigation step odom bag in {NAV_ROSBAG_DIR} directory (default: nav_odom4.bag)",
    # )
    
    args = parser.parse_args()
    main(args)


