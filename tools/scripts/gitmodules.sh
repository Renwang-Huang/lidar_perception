#!/bin/bash

GREEN='\033[0;32m'
NC='\033[0m' 

echo -e "${GREEN}Starting manual sync of Sophus repository...${NC}"

sync_repo() {
    local url=$1
    local path=$2

    if [ -d "$path/.git" ]; then
        echo -e "Directory $path already exists, attempting to update..."
        cd "$path" && git pull
        cd - > /dev/null
    else
        echo -e "Cloning $url into $path..."
        git clone --recursive "$url" "$path"
    fi
}

sync_repo "https://github.com/strasdat/Sophus.git" "3rdparty/Sophus"

echo -e "${GREEN}Sophus repository synchronized successfully!${NC}"

sync_repo "https://github.com/mavlink/mavros" "3rdparty/mavros"

echo -e "${GREEN}mavros repository synchronized successfully!${NC}"
