#! /usr/env/bin bash

# This line gets the name of the actual directory where the script is located,
# not just the working directory
SCRIPT_DIRNAME="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

bash ${SCRIPT_DIRNAME}/add_repo_sources.sh
bash ${SCRIPT_DIRNAME}/install_ros_desktop.sh
bash ${SCRIPT_DIRNAME}/install_reqd_pkgs.sh
