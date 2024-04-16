import hydra
from initializers import StartScript
import signal
import sys

@hydra.main(config_path="configs", config_name="start_script", version_base="1.2")
def main(cfg):
    """
    Main function to start the server
    """
    #Start the server
    server = StartScript(cfg)
    processes = server.get_processes()

    for process in processes:
        process.start()
    for process in processes:
        process.join()

    def signal_handler(sig, frame):
        for process in processes:
            process.terminate()
        sys.exit(0)
    signal.signal(signal.SIGINT, signal_handler)

if __name__ == "__main__":
    main()