BLUBBER_VARIANTS := buster bullseye

IMAGE = local/scap-$(*F)-test

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

image-%:
	rm -fr ./tests/scap/__pycache__
	@# Note: This copies all of the current directory into the container image
	DOCKER_BUILDKIT=1 docker build \
		--quiet \
		-f .pipeline/blubber.yaml \
		--target $(*F) \
		-t $(IMAGE) .

test-%: image-%
	docker run -it --rm $(IMAGE)

########################
# TARGETS
########################

.PHONY: images
images: $(BLUBBER_VARIANTS:%=image-%)

.PHONY: test
test: $(BLUBBER_VARIANTS:%=test-%)
