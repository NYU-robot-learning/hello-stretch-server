import hydra
from initializers import StickTeleop
import signal
import sys

@hydra.main(config_path="configs", config_name="stick_teleop", version_base="1.2")
def main(cfg):
    """
    Main function to start the server
    """
    #Start the server
    server = StickTeleop(cfg)
    processes = server.get_processes()

    for process in processes:
        process.start()
    for process in processes:
        process.join()
    print("Stick Teleop Started")

    def signal_handler(sig, frame):
        for process in processes:
            process.terminate()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    main()