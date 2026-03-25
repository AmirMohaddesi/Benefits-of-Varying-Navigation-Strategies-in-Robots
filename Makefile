.PHONY: test pydeps

pydeps:
	pip install -r requirements.txt

test: pydeps
	pytest tests/ -q
