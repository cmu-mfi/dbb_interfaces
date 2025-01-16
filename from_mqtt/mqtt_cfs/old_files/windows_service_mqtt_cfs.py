import win32serviceutil
import win32service
import win32event
import servicemanager
import logging
import os
import subprocess
import time

class SimplePythonService(win32serviceutil.ServiceFramework):
    _svc_name_ = "mqtt_cfs"
    _svc_display_name_ = "mqtt_cfs"
    _svc_description_ = "Run mqtt_cfs script as a service"

    def __init__(self, args):
        win32serviceutil.ServiceFramework.__init__(self, args)
        self.hWaitStop = win32event.CreateEvent(None, 0, 0, None)
        self.running = True
        self.setup_logging()

    def setup_logging(self):
        # Set up the logging directory and file
        log_directory = r"C:\MFI-Software\dbb_interfaces\from_mqtt\mqtt_cfs"
        os.makedirs(log_directory, exist_ok=True)
        log_file = os.path.join(log_directory, "log_windows_service_mqtt_cfs.log")

        # Configure the logging settings
        logging.basicConfig(
            filename=log_file,
            level=logging.DEBUG,
            format="%(asctime)s - %(levelname)s - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )
        logging.info("Logging setup complete.")

    def SvcStop(self):
        logging.info("Service stop request received.")
        self.ReportServiceStatus(win32service.SERVICE_STOP_PENDING)
        win32event.SetEvent(self.hWaitStop)
        self.running = False
        logging.info("Service is stopping...")

    def SvcDoRun(self):
        servicemanager.LogMsg(
            servicemanager.EVENTLOG_INFORMATION_TYPE,
            servicemanager.PYS_SERVICE_STARTED,
            (self._svc_name_, "")
        )
        logging.info("Service started successfully.")
        self.run_external_script()

    def run_external_script(self):
        try:
            # Define the path to the virtual environment and the script
            venv_path = r"C:\MFI-Software\dbb_interfaces\from_mqtt\mqtt_cfs\.venv\Scripts\python.exe"  # Update this path
            script_path = r"C:\MFI-Software\dbb_interfaces\from_mqtt\mqtt_cfs\mqtt_cfs.py"  # Update this path

            logging.info(f"Running the external script: {script_path} using venv: {venv_path}")

            # Use subprocess to call the external Python script within the virtual environment
            process = subprocess.Popen(
                [venv_path, script_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )

            # Read the output and error streams from the script
            stdout, stderr = process.communicate()

            if stdout:
                logging.info(f"Script output:\n{stdout}")
            if stderr:
                logging.error(f"Script error:\n{stderr}")

        except Exception as e:
            logging.error(f"An error occurred while running the external script: {e}", exc_info=True)
        finally:
            logging.info("External script execution finished.")

if __name__ == '__main__':
    win32serviceutil.HandleCommandLine(SimplePythonService)
