BLUBBER_VARIANTS := buster bullseye

VERIFY_DEPS_IMAGE = local/scap-$(*F)-verify-deps
TEST_IMAGE = local/scap-$(*F)-test

.PHONY: default
default:
	$(info This Makefile does not have a default target.)
	$(info Targets:)
	$(info `make images` Build the images)
	$(info `make test` Run all images (run tests))
	$(error exiting...)

########################
# RULES
########################

test-image-%:
	rm -fr ./tests/scap/__pycache__
	@# Note: This copies all of the current directory into the container image
	DOCKER_BUILDKIT=1 docker build \
		--quiet \
		-f .pipeline/blubber.yaml \
		--target verify-deps-$(*F) \
		-t $(VERIFY_DEPS_IMAGE) .
	DOCKER_BUILDKIT=1 docker build \
		--quiet \
		-f .pipeline/blubber.yaml \
		--target test-$(*F) \
		-t $(TEST_IMAGE) .

test-%: test-image-%
	docker run -it --rm $(TEST_IMAGE)

binary-dist-image-%:
	DOCKER_BUILDKIT=1 docker build \
		-f .pipeline/blubber.yaml \
		--target binary-dist-$* \
		-t local/scap-dist-$* .

########################
# TARGETS
########################

.PHONY: test-images
images: $(BLUBBER_VARIANTS:%=test-image-%)

.PHONY: test
test: $(BLUBBER_VARIANTS:%=test-%)

.PHONY: dist-images
dist-images: $(BLUBBER_VARIANTS:%=binary-dist-image-%)

.PHONY: reformat
reformat: test-image-bullseye
	docker run -it --rm --entrypoint black --user $$(id -u):$$(id -g) -v $$(pwd):/srv/app local/scap-bullseye-test . bin/scap
