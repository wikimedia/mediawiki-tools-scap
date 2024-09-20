BLUBBER_VARIANTS := buster bullseye bookworm
# deployment.eqiad.wmnet currently runs buster
DEFAULT_VARIANT := buster

VERIFY_DEPS_IMAGE = local/scap-$(*F)-verify-deps
TEST_IMAGE = local/scap-$(*F)-test

.PHONY: default
default:
	$(info This Makefile does not have a default target.)
	$(info Targets:)
	$(info `make test`     Run tests for $(DEFAULT_VARIANT))
	$(info `make test-all` Run tests for $(BLUBBER_VARIANTS))
	$(info `make reformat` Reformat source code using 'black' formatter)
	$(error exiting...)

########################
# RULES
########################

test-image-%:
	rm -fr ./tests/scap/__pycache__
	@# Note: This copies all of the current directory into the container image
	DOCKER_BUILDKIT=1 docker build \
		-f .pipeline/blubber.yaml \
		--target verify-deps-$(*F) \
		-t $(VERIFY_DEPS_IMAGE) .
	DOCKER_BUILDKIT=1 docker build \
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

.PHONY: test-all
test-all: $(BLUBBER_VARIANTS:%=test-%)

.PHONY: test
test: test-$(DEFAULT_VARIANT)

.PHONY: dist-images
dist-images: $(BLUBBER_VARIANTS:%=binary-dist-image-%)

.PHONY: reformat
reformat:
	tox -e reformat
