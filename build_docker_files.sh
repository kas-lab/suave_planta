# build suave_runner
docker build -t suave_runner -f $HOME/suave_ws/src/suave/suave_runner/docker/Dockerfile .

# build suave_rosa
docker build --build-arg BASE_IMAGE=suave_runner -t suave_rosa -f $HOME/suave_ws/src/suave_rosa/docker/Dockerfile-headless $HOME/suave_ws/src/suave_rosa/

# build suave_planta
docker build --build-arg BASE_IMAGE=suave_rosa -t suave_planta -f $HOME/suave_ws/src/suave_planta/Dockerfile .
