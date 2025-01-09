# SpiderPig

Welcome!

## Development requirements

* npm
* Docker Compose

## Running the SpiderPig API server

The SpiderPig user interface communicates with an API server to start, interact with, and retrieve information about MediaWiki deployment jobs. If you have Docker Compose installed on your computer, you can easily run an api server which will run simulated backports for testing.
 
### Start the API server

In the top level directory of the scap git repo:

```sh
local-dev/start-apiserver
```

The apiserver will listen on local port 8000.

### Get 2FA one-time-password information

```sh
local-dev/get-otp
```

### Stop the API server

```sh
local-dev/stop-apiserver
```

### Remove all traces of the API server

```sh
local-dev/clean-apiserver
```

## Access the SpiderPig UI

When you start the API as described in the prior section, `npm run build` is executed to build a static version of the UI.  You can access this static version of the UI at http://localhost:8000.

## Authentication

To log in, the username is `deployer01` and the password is `scap100`.  When you are prompted for a run time password, run `local-dev/get-otp` to generate it.

## Hot-Reload for Development

To avoid having to run `npm run build` after each change to a SpiderPig UI file, you can start a development webserver which will automatically handle reloads.

In this `web` directory:

```sh
npm run dev
```

Then visit http://localhost:5173 in your browser.
