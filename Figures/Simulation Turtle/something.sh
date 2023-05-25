#!/bin/bash
for FILE in ./*.pdf; do
  pdfcrop --margins '5 5 5 5' "${FILE}"
done
