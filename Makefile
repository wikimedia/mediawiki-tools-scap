default:
	$(error no default target. Try `make test`)

test:
	rm -fr ./tests/scap/__pycache__
	curl -o Dockerfile.tests -sf https://blubberoid.wikimedia.org/v1/test --data-binary @.pipeline/blubber.yaml  -H 'content-type: application/yaml'
	@# Note: This copies all of the current directory into the container image
	docker build -f Dockerfile.tests -t local/scap-test --quiet .
	docker run -it --rm local/scap-test
