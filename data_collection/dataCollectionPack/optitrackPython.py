
# ============================================================================= #type: ignore  # noqa E501
# Copyright © 2025 NaturalPoint, Inc. All Rights Reserved.
#
# THIS SOFTWARE IS GOVERNED BY THE OPTITRACK PLUGINS EULA AVAILABLE AT https://www.optitrack.com/about/legal/eula.html #type: ignore  # noqa E501
# AND/OR FOR DOWNLOAD WITH THE APPLICABLE SOFTWARE FILE(S) (“PLUGINS EULA”). BY DOWNLOADING, INSTALLING, ACTIVATING #type: ignore  # noqa E501
# AND/OR OTHERWISE USING THE SOFTWARE, YOU ARE AGREEING THAT YOU HAVE READ, AND THAT YOU AGREE TO COMPLY WITH AND ARE #type: ignore  # noqa E501
# BOUND BY, THE PLUGINS EULA AND ALL APPLICABLE LAWS AND REGULATIONS. IF YOU DO NOT AGREE TO BE BOUND BY THE PLUGINS #type: ignore  # noqa E501
# EULA, THEN YOU MAY NOT DOWNLOAD, INSTALL, ACTIVATE OR OTHERWISE USE THE SOFTWARE AND YOU MUST PROMPTLY DELETE OR #type: ignore  # noqa E501
# RETURN IT. IF YOU ARE DOWNLOADING, INSTALLING, ACTIVATING AND/OR OTHERWISE USING THE SOFTWARE ON BEHALF OF AN ENTITY, #type: ignore  # noqa E501
# THEN BY DOING SO YOU REPRESENT AND WARRANT THAT YOU HAVE THE APPROPRIATE AUTHORITY TO ACCEPT THE PLUGINS EULA ON #type: ignore  # noqa E501
# BEHALF OF SUCH ENTITY. See license file in root directory for additional governing terms and information. #type: ignore  # noqa E501
# ============================================================================= #type: ignore  # noqa E501


# OptiTrack NatNet direct depacketization sample for Python 3.x
#
# Uses the Python NatNetClient.py library to establish
# a connection and receive data via that NatNet connection
# to decode it using the NatNetClientLibrary.

import sys
import time
import argparse
from functools import partial
from NatNetClient import NatNetClient
import DataDescriptions
import MoCapData

# This is a callback function that gets connected to the NatNet client
# and called once per mocap frame.


def receive_new_frame(data_dict):
    dump_args = False
    if dump_args is True:
        out_string = "    "
        for key in data_dict:
            out_string += key + "= "
            if key in data_dict:
                out_string += data_dict[key] + " "
            out_string += "/"
        print(out_string)


def receive_new_frame_with_data(file, data_dict):
    # TODO: implement logging code here
    # keys_list = [
    #     "frame_number", "marker_set_count", "unlabeled_markers_count", "rigid_body_count", 
    #     "skeleton_count", "asset_count", "labeled_marker_count", "timecode", "timecode_sub", 
    #     "timestamp", "is_recording", "tracked_models_changed", "offset", "mocap_data"]
    # dump_args = True
    # if dump_args is True:
    #     out_string = "    "
    #     for key in data_dict:
    #         out_string += key + "= "
    #         if key in data_dict:
    #             out_string += str(data_dict[key]) + " "
    #         out_string += "/"
    #     print(out_string)

    

    # file.write(f"{data_dict['timestamp']}")  # timestamp of the mocap system
    file.write(f"{time.time()}")

    # Example data access:
    body_data = data_dict["mocap_data"].rigid_body_data
    # Sort rigid bodies by id_num (frame id)
    sorted_rigid_bodies = sorted(body_data.rigid_body_list, key=lambda item: item.id_num)
    for item in sorted_rigid_bodies:
        id = item.id_num
        is_valid = item.tracking_valid
        pos = item.pos
        rot = item.rot
        file.write(f",{id},{int(is_valid)},{pos[0]},{pos[1]},{pos[2]},{rot[0]},{rot[1]},{rot[2]},{rot[3]}")
    file.write("\n")



# This is a callback function that gets connected to the NatNet client.
# It is called once per rigid body per frame.
def receive_rigid_body_frame(new_id, position, rotation):
    pass
    # print("Received frame for rigid body", new_id)
    # print("Received frame for rigid body", new_id," ",position," ",rotation)


def add_lists(totals, totals_tmp):
    totals[0] += totals_tmp[0]
    totals[1] += totals_tmp[1]
    totals[2] += totals_tmp[2]
    return totals


def print_configuration(natnet_client):
    natnet_client.refresh_configuration()
    print("Connection Configuration:")
    print("  Client:          %s" % natnet_client.local_ip_address)
    print("  Server:          %s" % natnet_client.server_ip_address)
    print("  Command Port:    %d" % natnet_client.command_port)
    print("  Data Port:       %d" % natnet_client.data_port)

    changeBitstreamString = "  Can Change Bitstream Version = "
    if natnet_client.use_multicast:
        print("  Using Multicast")
        print("  Multicast Group: %s" % natnet_client.multicast_address)
        changeBitstreamString += "false"
    else:
        print("  Using Unicast")
        changeBitstreamString += "true"

    # NatNet Server Info
    application_name = natnet_client.get_application_name()
    nat_net_requested_version = natnet_client.get_nat_net_requested_version()
    nat_net_version_server = natnet_client.get_nat_net_version_server()
    server_version = natnet_client.get_server_version()

    print("  NatNet Server Info")
    print("    Application Name %s" % (application_name))
    print("    MotiveVersion  %d %d %d %d" % (server_version[0], server_version[1], server_version[2], server_version[3]))  # type: ignore  # noqa F501
    print("    NatNetVersion  %d %d %d %d" % (nat_net_version_server[0], nat_net_version_server[1], nat_net_version_server[2], nat_net_version_server[3]))  # type: ignore  # noqa F501
    print("  NatNet Bitstream Requested")
    print("    NatNetVersion  %d %d %d %d" % (nat_net_requested_version[0], nat_net_requested_version[1],  # type: ignore  # noqa F501
                                              nat_net_requested_version[2], nat_net_requested_version[3]))  # type: ignore  # noqa F501

    print(changeBitstreamString)
    # print("command_socket = %s" % (str(natnet_client.command_socket)))
    # print("data_socket    = %s" % (str(natnet_client.data_socket)))
    print("  PythonVersion    %s" % (sys.version))


def print_commands(can_change_bitstream):
    outstring = "Commands:\n"
    outstring += "Return Data from Motive\n"
    outstring += "  s  send data descriptions\n"
    outstring += "  r  resume/start frame playback\n"
    outstring += "  p  pause frame playback\n"
    outstring += "     pause may require several seconds\n"
    outstring += "     depending on the frame data size\n"
    outstring += "Change Working Range\n"
    outstring += "  o  reset Working Range to: start/current/end frame 0/0/end of take\n"  # type: ignore  # noqa F501
    outstring += "  w  set Working Range to: start/current/end frame 1/100/1500\n"  # type: ignore  # noqa F501
    outstring += "Return Data Display Modes\n"
    outstring += "  j  print_level = 0 supress data description and mocap frame data\n"  # type: ignore  # noqa F501
    outstring += "  k  print_level = 1 show data description and mocap frame data\n"  # type: ignore  # noqa F501
    outstring += "  l  print_level = 20 show data description and every 20th mocap frame data\n"  # type: ignore  # noqa F501
    outstring += "Change NatNet data stream version (Unicast only)\n"
    outstring += "  3  Request NatNet 3.1 data stream (Unicast only)\n"
    outstring += "  4  Request NatNet 4.1 data stream (Unicast only)\n"
    outstring += "General\n"
    outstring += "  t  data structures self test (no motive/server interaction)\n"  # type: ignore  # noqa F501
    outstring += "  c  print configuration\n"
    outstring += "  h  print commands\n"
    outstring += "  q  quit\n"
    outstring += "\n"
    outstring += "NOTE: Motive frame playback will respond differently in\n"
    outstring += "       Endpoint, Loop, and Bounce playback modes.\n"
    outstring += "\n"
    outstring += "EXAMPLE: PacketClient [serverIP [ clientIP [ Multicast/Unicast]]]\n"  # type: ignore  # noqa F501
    outstring += "         PacketClient \"192.168.10.14\" \"192.168.10.14\" Multicast\n"  # type: ignore  # noqa F501
    outstring += "         PacketClient \"127.0.0.1\" \"127.0.0.1\" u\n"
    outstring += "\n"
    print(outstring)


def request_data_descriptions(s_client):
    # Request the model definitions
    s_client.send_request(s_client.command_socket, s_client.NAT_REQUEST_MODELDEF, "",  (s_client.server_ip_address, s_client.command_port))  # type: ignore  # noqa F501


def test_classes():
    totals = [0, 0, 0]
    print("Test Data Description Classes")
    totals_tmp = DataDescriptions.test_all()
    totals = add_lists(totals, totals_tmp)
    print("")
    print("Test MoCap Frame Classes")
    totals_tmp = MoCapData.test_all()
    totals = add_lists(totals, totals_tmp)
    print("")
    print("All Tests totals")
    print("--------------------")
    print("[PASS] Count = %3.1d" % totals[0])
    print("[FAIL] Count = %3.1d" % totals[1])
    print("[SKIP] Count = %3.1d" % totals[2])


if __name__ == "__main__":


    # Set up command-line argument parsing
    parser = argparse.ArgumentParser(description="Read samples from Vicon")
    parser.add_argument('duration', type=int,
                        help="Duration to run the data collection (in seconds)")
    parser.add_argument('filename', type=str, help="Filename to save the data")
    parser.add_argument('--start-time', type=float, default=None,
                        help="Shared start timestamp (seconds)")
    args = parser.parse_args()

    optionsDict = {}
    optionsDict["clientAddress"] = "1.1.1.2" # Your computer IP 
    optionsDict["serverAddress"] = "1.1.1.1" # Host computer IP 
    optionsDict["use_multicast"] = False
    optionsDict["stream_type"] = 'd'  # c or d
    stream_type_arg = None

    # This will create a new NatNet client
    streaming_client = NatNetClient()
    streaming_client.set_client_address(optionsDict["clientAddress"])
    streaming_client.set_server_address(optionsDict["serverAddress"])
    streaming_client.set_use_multicast(optionsDict["use_multicast"])

    # Streaming client configuration.
    # Calls RB handler on emulator for data transmission.
    # streaming_client.new_frame_listener = receive_new_frame
    f = open(args.filename, 'w')

    disks = 5
    header = "timestamp_s,"

    data_disk = ["frame_number","is_valid","x","y","z","qx","qy","qz","qw"]
    for it in range(0, 5):
        disk = "disk_" + str(it) + "_"

        header_disk = ""
        for data in data_disk:
            header_disk += disk + data + ","

        header += header_disk

    
    header = header.rstrip(",") + "\n"

    # f.write("timestamp (s),frame number,is valid,x,y,z,qx,qy,qz,qw\n")
    f.write(header)
    streaming_client.new_frame_with_data_listener = partial(receive_new_frame_with_data, f)  # type ignore # noqa E501
    # streaming_client.rigid_body_listener = receive_rigid_body_frame

    # print instructions
    print("NatNet Python Client 4.4\n")

    # Start up the streaming client now that the callbacks are set up.
    # This will run perpetually, and operate on a separate thread.
    is_running = streaming_client.run(optionsDict["stream_type"])
    if not is_running:
        print("ERROR: Could not start streaming client.")
        try:
            sys.exit(1)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    is_looping = True
    time.sleep(1)
    if streaming_client.connected() is False:
        print("ERROR: Could not connect properly.  Check that Motive streaming is on.")  # type: ignore  # noqa F501
        try:
            sys.exit(2)
        except SystemExit:
            print("...")
        finally:
            print("exiting")

    print_configuration(streaming_client)
    print("\n")
    print_commands(streaming_client.can_change_bitstream_version())

    if args.start_time is not None:
        print(f"Using shared start time {args.start_time:.6f}")
        start_time = args.start_time
    else:
        start_time = time.time()
    time.sleep(args.duration)

    print("Finished data collection, shutting down client...")
    streaming_client.shutdown()
    f.close()
    print("exiting")
