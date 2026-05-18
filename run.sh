xhost +local:docker
docker compose up -d --build
./connect.sh
