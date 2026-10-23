# Run latexmk from this directory. Keep all compiler output in .build,
# and publish only the finished PDF beside its source after a successful build.
$pdf_mode = 5;
$out_dir = '.build';
$aux_dir = '.build';
$success_cmd = 'cp -- %Z%R.pdf %R.pdf';
