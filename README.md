# dronedoc

## Tutorial
### 日本語版
チュートリアルはhttps://uenota.github.io/dronedoc/ja/index.html からアクセスできます。  
問題がある場合は[Issues](https://github.com/uenota/dronedoc/issues)からお知らせください。

### English
Tutorials are available at https://uenota.github.io/dronedoc/en/index.html.  
Please let me know if you have any problems from [Issues](https://github.com/uenota/dronedoc/issues).

**NOTE: This tutorial is not fully translated into English.**

## Docker
You can set up environment with docker instead of building by yourself.

### Build Docker Image
You can use `docker/tutorial/build.sh` to build docker image.
The following command will build docker image for tutorial.

```bash
./build.sh --imname uenot/dronedoc
```

This command will build docker image with name of `uenot/dronedoc`.

You can change the name of docker image by changing argument of `--imname` option. 

### Run Docker Container
You can use `docker/tutorial/run.sh` to run docker container.
The following command will start up docker container for tutorial.
Note that this container will be removed after exiting from the container.

```bash
./run.sh --imname uenot/dronedoc
```

This command will run docker container from image with name of `uenot/dronedoc`.

If your machine have GPU, add `--gpu` option.
```bash
./run.sh --imname uenot/dronedoc --gpu
```

### References
See http://wiki.ros.org/docker/Tutorials/GUI#The_isolated_way for the details of Dockerfile and run.sh script.

## License
<a rel="license" href="http://creativecommons.org/licenses/by/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by/4.0/">Creative Commons Attribution 4.0 International License</a>.
