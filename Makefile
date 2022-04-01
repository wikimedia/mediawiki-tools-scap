default:
	$(error no default target. Try `make test`)

test:
	rm -fr ./tests/scap/__pycache__
	@# Note: This copies all of the current directory into the container image
	DOCKER_BUILDKIT=1 docker build -f .pipeline/blubber.yaml -t local/scap-test --quiet .
	docker run -it --rm local/scap-test
