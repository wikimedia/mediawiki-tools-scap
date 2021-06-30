default:
	@echo no default target. Try make test
	false

test:
	rm -fr ./tests/scap/__pycache__
	curl -sf https://blubberoid.wikimedia.org/v1/test --data-binary @.pipeline/blubber.yaml  -H 'content-type: application/yaml' > Dockerfile.tests
	@# Note: This copies all of the current directory into the container image
	docker build -f Dockerfile.tests --iidfile id .
	docker run --rm "$$(cat id)"
	rm id
