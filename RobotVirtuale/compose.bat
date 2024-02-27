docker compose down

docker rmi app/action_image
docker build -t app/action_image action/

docker rmi app/sense_image
docker build -t app/sense_image sense/

docker rmi app/perc_image
docker build -t app/perc_image perception/

start python gesture.py

docker rmi app/contr_image
docker build -t app/contr_image controller/

docker compose up
