# SpiderPig

This template should help get you started developing with Vue 3 in Vite.

## Recommended IDE Setup

[VSCode](https://code.visualstudio.com/) + [Volar](https://marketplace.visualstudio.com/items?itemName=Vue.volar) (and disable Vetur).

## Customize configuration

See [Vite Configuration Reference](https://vitejs.dev/config/).

## Project Setup

```sh
npm install
```

### Compile and Hot-Reload for Development

```sh
npm run dev
```

### Compile and Minify for Production

```sh
npm run build
```

### Set up Scap for local development

In the top level directory:

```sh
(cd web && npm run build)
python3 -m venv /tmp/scap-venv
source /tmp/scap-venv/bin/activate
pip install -e .
```

### Start the SpiderPig Jobrunner and API server

This assumes that you have previously activated a Python virtualenv
with Scap installed in it.

```sh
scap spiderpig-jobrunner -Dlocal_dev_mode:true -Dstage_dir:/tmp/fakestaging &
scap spiderpig-apiserver --dev -Dlocal_dev_mode:true -Dstage_dir:/tmp/fakestaging &
```
The apiserver will listen on local port 8000.

### One time password generation

If you are prompted for username and password in the SpiderPig UI, run the following to retrieve that information:
```sh
scap spiderpig-otp -Dlocal_dev_mode:True -Dstage_dir:/tmp/fakestaging
```

The jobrunner and apiserver can be killed when you're done with development/testing.
