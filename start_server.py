import hydra
from initializers import Start_Server

@hydra.main(config_path="configs", config_name="start_server", version_base="1.2")
def main(cfg):
    """
    Main function to start the server
    """
    #Start the server
    server = Start_Server(cfg)
    processes = server.get_processes()

    for process in processes:
        process.start()
    for process in processes:
        process.join()
    print("Server started")

if __name__ == "__main__":
    main()