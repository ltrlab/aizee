# Table fiducial labels

Fiducial tags stuck on the robot table give fixed reference points for the head
and wrist camera extrinsics ([calibration order](CALIBRATION.md), step 11) and for
measuring where the table and bucket really are. The tag positions are recorded in
`config/devices/minerva-table-board.json`; the simulator's workcell positions are in
`config/devices/simulation/minerva-production-01-workcell.json` ([workcell](../simulation/SIM_WORKCELL.md)).
The labels are printed on a Brother QL-800 label printer, 300 dpi.

**Status.** The labels were printed and stuck on this unit's table inside a taped rectangle (2026-10-17 and 18). Their layout
as it was measured (to about +-5 mm, by eye and tape) is `config/devices/minerva-table-board.json`, in the taped rectangle's own
frame; the black square measures 43.18 mm by calipers, and the head camera's fit of tags 0 and 2 to 5 at 0.75 px RMS confirmed
their orientation. The placement plan and measurement tables below are the original plan for the table's own frame; fill the
tables in only if you want tag positions in that frame. A new table or a re-stuck label means re-measuring the board file.

## Printer and media

- **Printer.** A Brother QL-800 on a Windows PC with Brother's printer driver (1.10.1.0,
  `bsq16av.inf`) installed, which makes the "Brother QL-800" queue. Raw USB printing
  (Zadig/WinUSB, brother_ql) is not used.
- **Media.** The driver offers every QL roll and cannot report which one is
  loaded. Its default page, 29 x 90 mm die-cut in landscape, is a fixed
  default, not a reading of the roll. Read the part number on the roll. The
  default design is for 62 mm tape (DK-22205 or DK-22251 continuous, 696
  printable dots across); it also fits the 62 x 100 mm die-cut DK-11202. A
  29 mm variant (DK-22210, or DK-11201 29 x 90 mm, 306 dots) is for the wrist
  cameras only.

## Family and size

The tape width, not the family, sets the largest tag. With a white quiet zone
of one cell on each side and 1 mm to spare, a family with g cells across
(black border included) gets floor((696 - 12) / (g + 2)) dots per cell:

| Family | Cells across | Dots per cell | Cell | Black square |
|---|---|---|---|---|
| ArUco DICT_4X4_50 | 6 | 85 | 7.197 mm | 43.18 mm |
| AprilTag tag25h9 (or DICT_5X5) | 7 | 76 | 6.435 mm | 45.04 mm |
| AprilTag tag36h11 (or DICT_6X6) | 8 | 68 | 5.757 mm | 46.06 mm |

Taking the head camera's 69 degree field of view over the 640 px row gives
fx = 320 / tan(34.5 deg) = 466 px, so a cell of c mm at Z m spans
466 c / (1000 Z) px:

| Family | 0.6 m | 0.9 m | 1.2 m |
|---|---|---|---|
| DICT_4X4_50 | 5.6 | 3.7 | 2.8 px per cell |
| tag25h9 | 5.0 | 3.3 | 2.5 |
| tag36h11 | 4.5 | 3.0 | 2.2 |

A tilted view shortens one axis by cos(tilt), and decoding falls off below
about 3 px per cell, faster for families with more cells. So the family with
the fewest cells wins at the far end of the range. **The labels use
DICT_4X4_50 with 85 dots (7.197 mm) per cell: a 510 dot, 43.18 mm black
square.** Its weaker code (one-bit correction, 50 IDs) matters little here:
only IDs 0-7 are used, at measured positions, and a detector can drop any
other ID.

The 69 degree figure is conservative. A D435 normally delivers 640x480 colour
as a 4:3 crop of its 1920x1080 sensor, typically fx of about 616 px (about
55 degrees), which gives a third more pixels per cell. Read this camera's
intrinsics before calibrating.

`scripts/fiducials/check_tag_range.py` renders each 62 mm label into a camera
view (supersampled, lens blur, sensor noise, JPEG) and counts default ArUco
detector decodes of the right ID, 20 trials per point at a random rotation,
seed 20261017:

| View, family | Tilt | 0.6 m | 0.9 m | 1.2 m | 1.5 m |
|---|---|---|---|---|---|
| 640x480, fx 466, DICT_4X4_50 | 0 / 45 / 60 deg | 20 / 20 / 20 | 20 / 20 / 15 | 20 / 15 / 2 | 16 / 9 / 0 |
| 640x480, fx 466, tag25h9 | 0 / 45 / 60 deg | 20 / 20 / 20 | 20 / 19 / 10 | 18 / 6 / 1 | 8 / 0 / 0 |
| 640x480, fx 466, tag36h11 | 0 / 45 / 60 deg | 20 / 20 / 19 | 20 / 14 / 1 | 10 / 1 / 0 | 0 / 0 / 0 |
| 640x480, fx 616, DICT_4X4_50 | 0 / 45 / 60 deg | 20 / 20 / 20 | 20 / 20 / 20 | 20 / 20 / 18 | 20 / 16 / 9 |
| 1920x1080, fx 1397, DICT_4X4_50 | 0 / 45 / 60 deg | 20 / 20 / 20 | 20 / 20 / 20 | 20 / 20 / 20 | 20 / 20 / 20 |

This is a model without lens distortion, glare or motion blur. It ranks the
families; it does not replace a look through the real camera. If the far
tags do not decode at 640x480, take calibration frames at 1280x720 or
1920x1080 rather than changing the family.

The 29 mm variant (36 dots = 3.048 mm per cell, an 18.29 mm square) gives 1.2
px per cell at 1.2 m. It is for the wrist cameras, which come within a few
tens of centimetres of the table.

## The labels

`python scripts/fiducials/make_tag_labels.py` writes
`build/fiducials/62-DICT_4X4_50-c85/id00.png` to `id07.png` (1-bit, 300 dpi,
696 x 847 dots = 58.93 x 71.71 mm) and `labels.pdf`, one exact-size page per
label. Each label has:

- the tag, drawn dot for dot, in a white square as wide as the printable area
  (93 dots, 1.09 cells, of quiet zone at the sides, plus about 1.5 mm of
  unprinted tape);
- "ID n ArUco DICT_4X4_50", "black square 43.18 mm" and "text end toward
  robot";
- an L-shaped scale whose arms run 10 mm across and along the tape from their
  shared outer corner.

The generator checks every label before writing it: the tag matches the
dictionary dot for dot, the one-cell quiet zone is white, and OpenCV decodes
the label's own ID with corners within 1.5 dots of the drawn square.
`--media 29` (or 38, 50, 29x90, 62x100) sizes the same design for another
roll; `--ids` and `--family` choose the tags.

## Placement plan

**Table frame.** Stand behind the robot, facing the table as the robot does.
The origin is the near-left corner of the table top, at the outer edge of the
silver trim. x runs to the right along the near long edge (0 to about 1524 mm),
y runs away from the robot (0 to about 762 mm), z is up.

Stick every label flat, with its text end toward the robot and its edges
parallel to the table edges. Each tag's OpenCV marker frame (x right, y up,
z out of the tag) is then parallel to the table frame, so a tag's pose is
just its centre (x, y, 0).

| ID | Place | Nominal tag centre (x, y) |
|---|---|---|
| 0 | near-left corner | 100, 100 mm |
| 1 | near-right corner | 1424, 100 mm |
| 2 | far-right corner | 1424, 662 mm |
| 3 | far-left corner | 100, 662 mm |
| 4 | far edge, middle | 762, 662 mm |
| 5 | near edge, middle | 762, 100 mm |
| 6 | in front of the bucket's near-left corner, label 1 cm from the bucket | measured |
| 7 | in front of the bucket's near-right corner, label 1 cm from the bucket | measured |

On the label the tag centre sits 31 mm from each side, about 32 mm from the far
end and about 45 mm from the text end, so at 100 mm from both edges every label
stays about 55 mm or more inside the outer edge, clear of the trim. The exact
place does not matter; the measured one does.

The head camera looks forward over the table. In a recording-proof frame taken on 2026-10-17
the far edge and its trim cross the whole image, the near edge is below the
image, and the far corners are at or past its sides. Tags 0, 1 and 5 are
likely outside the head view and serve the wrist cameras; 2, 3, 4 and the
bucket tags serve the head camera. Lay all eight out before sticking them and
look at Helm's head camera view. If 2 or 3 is not wholly inside the image,
slide it inward along the far edge until it is, with about a label's width to
spare.

## What to measure

After sticking the labels down:

1. **One label's print scale, before sticking it.** Measure the black square
   across and along the tape, and the two 10 mm arms. Expect 43.18 mm both
   ways. A different length along the tape is the printer's feed scale; record
   it, since the detector takes one square size.
2. **The table.** Its outer length and width and the trim width (the workcell
   file has "about 5 ft by 2.5 ft" and a placeholder trim).
3. **Each tag.** From the table's left outer edge to the black square's left and
   right edges (x), and from the near outer edge to its near and far edges (y).
   The centre is the mean of each pair. The differences re-check the 43.18 mm
   square, and measuring both ends of one edge shows any rotation.
4. **The bucket.** Its four outer corners in the same frame, and its height.
5. **The robot against the table.** The near edge's distance d in front of the
   hanging arms, and the x of the robot's centreline c (midway between the two
   hanging arms) along the near edge. These are the workcell's "table position"
   placeholders.

The workcell file uses the Minerva base frame (x toward the robot's left, y
backward, z up) and puts the hanging arms at y = 0. If the table is square to
the robot, a point at table (x, y) is at base (c - x, -d - y) on the table top.
With the file's placeholders (c = 762 mm, d = 80 mm) the table centre maps to
(0, -0.461 m), as the file has it.

| ID | x (mm) | y (mm) | Notes |
|---|---|---|---|
| 0 | | | |
| 1 | | | |
| 2 | | | |
| 3 | | | |
| 4 | | | |
| 5 | | | |
| 6 | | | |
| 7 | | | |

## Printing and reprinting

The generator needs OpenCV 4.7 or newer, which the hash-locked dev venv does
not include; use a Python install that has it.

1. Read the roll's part number. `-Media` takes `62` for DK-22205 or DK-22251
   (62 mm continuous) and `62x100` for DK-11202 (62 x 100 mm die-cut).
2. From a PowerShell prompt in the repository root, check what the driver will use:
   `.\scripts\fiducials\print_label.ps1 -Media 62 -DryRun -Image build\fiducials\62-DICT_4X4_50-c85\id00.png`.
   For `-Media 62` it reports media code 259, a 62 x 78.7 mm portrait page at
   300x300 dpi, printable 58.93 mm (696 dots) across.
3. Print the test label (ID 0) and measure it (step 1 of "What to measure"):
   `.\scripts\fiducials\print_label.ps1 -Media 62 -Image build\fiducials\62-DICT_4X4_50-c85\id00.png`.
4. If it measures right, print the rest:
   `.\scripts\fiducials\print_label.ps1 -Media 62 -Image (Get-ChildItem build\fiducials\62-DICT_4X4_50-c85\id0[1-7].png).FullName`.

The helper selects the driver's own paper for the roll, because the QL-800
driver ignores a custom page size and falls back to 29 x 90 mm landscape. It
draws each image at its pixel size over 300 dpi with nearest-neighbour
sampling, centred in the printable area in portrait, with no fit-to-page. It
reads back the driver's print ticket and prints nothing unless the resolution
is a multiple of 300 dpi and the image fits the printable area. A job for the
wrong roll makes the printer stop with an error rather than print.

`-PdfFile out.pdf` sends the same drawing through Microsoft Print to PDF
instead. There the test label came out unresampled at 58.93 x 71.71 mm, its
exact 300 dpi size. As a fallback, each page of `labels.pdf` is one label at
its exact size (58.93 x 71.71 mm, lossless 1-bit images). Print it at "Actual
size" (100%), never "Fit", with the driver's paper set to the roll.
