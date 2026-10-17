<#
.SYNOPSIS
Prints 300 dpi label images through a Windows printer driver at their exact size.

.DESCRIPTION
Each image is drawn at its pixel size divided by 300 dpi with nearest-neighbour
sampling, centred on a page as wide as the tape and as long as the image plus a
feed margin at each end. Nothing is fitted to the page, so on a 300 dpi printer
every image dot becomes one printer dot.

-DryRun lists what the driver offers (resolutions, paper sizes, printable area)
and the planned page without printing. -PdfFile prints one image through
Microsoft Print to PDF into a file instead, which checks the placement without
using tape.

.EXAMPLE
powershell -ExecutionPolicy Bypass -File scripts\fiducials\print_label.ps1 -DryRun -Image build\fiducials\62-DICT_4X4_50-c85\id00.png

.EXAMPLE
powershell -ExecutionPolicy Bypass -File scripts\fiducials\print_label.ps1 -Image build\fiducials\62-DICT_4X4_50-c85\id00.png
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string[]] $Image,
    [string] $Printer = 'Brother QL-800',
    [double] $TapeWidthMm = 62,
    # 0 means continuous tape: the image length plus FeedMarginMm at each end.
    # For a die-cut roll pass the label length, for example 100 for DK-11202.
    [double] $PageLengthMm = 0,
    [double] $FeedMarginMm = 3,
    [string] $PdfFile,
    [switch] $DryRun
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
$ImageDpi = 300.0

function ConvertTo-Hundredths([double] $Mm) { [int][math]::Round($Mm / 25.4 * 100) }
function Format-Mm([double] $Hundredths) { '{0:N2}' -f ($Hundredths * 0.254) }

if ($PdfFile) {
    if ($Image.Count -ne 1) { throw '-PdfFile takes one image.' }
    $Printer = 'Microsoft Print to PDF'
}
$settings = New-Object System.Drawing.Printing.PrinterSettings
$settings.PrinterName = $Printer
if (-not $settings.IsValid) {
    $installed = (Get-Printer | ForEach-Object { $_.Name }) -join ', '
    throw "Printer '$Printer' is not installed. Installed printers: $installed"
}
if ($PdfFile) {
    $settings.PrintToFile = $true
    $settings.PrintFileName = [IO.Path]::GetFullPath($PdfFile)
}

if ($DryRun) {
    "Printer      $Printer"
    'Resolutions  ' + (($settings.PrinterResolutions | ForEach-Object { "$($_.Kind) $($_.X)x$($_.Y)" }) -join ', ')
    'Paper sizes (mm):'
    foreach ($paper in $settings.PaperSizes) {
        '  {0,-32} {1,7} x {2,-7} raw kind {3}' -f $paper.PaperName, (Format-Mm $paper.Width), (Format-Mm $paper.Height), $paper.RawKind
    }
    $default = $settings.DefaultPageSettings
    "Default page $($default.PaperSize.PaperName); hard margins $(Format-Mm $default.HardMarginX) x $(Format-Mm $default.HardMarginY) mm; printable $(Format-Mm $default.PrintableArea.Width) x $(Format-Mm $default.PrintableArea.Height) mm"
}

foreach ($path in $Image) {
    $full = (Resolve-Path -LiteralPath $path).Path
    $name = [IO.Path]::GetFileName($full)
    $bitmap = [System.Drawing.Image]::FromFile($full)
    try {
        $widthIn = $bitmap.Width / $ImageDpi
        $heightIn = $bitmap.Height / $ImageDpi
        $lengthMm = if ($PageLengthMm -gt 0) { $PageLengthMm } else { $heightIn * 25.4 + 2 * $FeedMarginMm }
        '{0}: {1} x {2} dots = {3:N2} x {4:N2} mm, page {5} x {6:N1} mm' -f $name, $bitmap.Width, $bitmap.Height, ($widthIn * 25.4), ($heightIn * 25.4), $TapeWidthMm, $lengthMm
        if ($DryRun) { continue }

        $doc = New-Object System.Drawing.Printing.PrintDocument
        $doc.PrinterSettings = $settings
        $doc.DocumentName = "Fiducial label $name"
        $doc.PrintController = New-Object System.Drawing.Printing.StandardPrintController
        $doc.OriginAtMargins = $false
        if (-not $PdfFile) {
            $doc.DefaultPageSettings.PaperSize = New-Object System.Drawing.Printing.PaperSize('Fiducial label', (ConvertTo-Hundredths $TapeWidthMm), (ConvertTo-Hundredths $lengthMm))
        }
        $doc.DefaultPageSettings.Margins = New-Object System.Drawing.Printing.Margins(0, 0, 0, 0)
        $doc.add_PrintPage({
            param($sender, $e)
            $g = $e.Graphics
            $g.PageUnit = [System.Drawing.GraphicsUnit]::Inch
            $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::NearestNeighbor
            $g.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::Half
            $page = $e.PageSettings
            $paperW = $page.PaperSize.Width / 100.0
            $paperH = $page.PaperSize.Height / 100.0
            if ($page.Landscape) { $paperW, $paperH = $paperH, $paperW }
            # The drawing origin is the printable area's corner, HardMargin in from the paper's corner.
            # Snap the image's corner to a whole device dot so each image dot lands on printer dots.
            $x = [math]::Round((($paperW - $widthIn) / 2 - $page.HardMarginX / 100.0) * $g.DpiX) / $g.DpiX
            $y = [math]::Round((($paperH - $heightIn) / 2 - $page.HardMarginY / 100.0) * $g.DpiY) / $g.DpiY
            $g.DrawImage($bitmap, (New-Object System.Drawing.RectangleF([single]$x, [single]$y, [single]$widthIn, [single]$heightIn)))
            $clip = $g.VisibleClipBounds
            Write-Host ('  printed on {0} ({1:N2} x {2:N2} mm) at {3}x{4} dpi; image corner {5:N2}, {6:N2} mm from the printable corner; printable {7:N2} x {8:N2} mm' -f `
                $page.PaperSize.PaperName, ($paperW * 25.4), ($paperH * 25.4), $g.DpiX, $g.DpiY, ($x * 25.4), ($y * 25.4), ($clip.Width * 25.4), ($clip.Height * 25.4))
            if (($g.DpiX % $ImageDpi) -ne 0 -or ($g.DpiY % $ImageDpi) -ne 0) {
                Write-Warning "The driver renders at $($g.DpiX)x$($g.DpiY) dpi, not a multiple of 300; cells will be uneven by a dot."
            }
            if ($x -lt $clip.Left -or $y -lt $clip.Top -or ($x + $widthIn) -gt $clip.Right -or ($y + $heightIn) -gt $clip.Bottom) {
                Write-Warning 'The image runs past the printable area; check that only white margin is cut off.'
            }
            $e.HasMorePages = $false
        })
        $doc.Print()
    }
    finally {
        $bitmap.Dispose()
    }
}
