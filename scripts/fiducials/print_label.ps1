<#
.SYNOPSIS
Prints 300 dpi label images through a Windows printer driver at their exact size.

.DESCRIPTION
Each image is drawn at its pixel size divided by 300 dpi with nearest-neighbour
sampling, centred in the printable area of the driver's own paper for the loaded
roll, in portrait. Continuous tape gets a page as long as the image plus a feed
margin at each end. Nothing is fitted to the page, so on a 300 dpi printer every
image dot becomes one printer dot. Nothing is printed if the driver's resolution
is not a multiple of 300 dpi or the image does not fit its printable area.

-Media names the loaded DK roll as make_tag_labels.py does. The QL-800 driver
cannot tell which roll is loaded; read the part number on the roll. A job for
the wrong roll makes the printer stop with an error instead of printing.

-DryRun shows what the driver will use for each image (media, length,
orientation, resolution, printable area) without printing. -PdfFile prints one
image through Microsoft Print to PDF into a file instead.

.EXAMPLE
.\scripts\fiducials\print_label.ps1 -Media 62 -DryRun -Image build\fiducials\62-DICT_4X4_50-c85\id00.png

.EXAMPLE
.\scripts\fiducials\print_label.ps1 -Media 62 -Image build\fiducials\62-DICT_4X4_50-c85\id00.png
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)] [string[]] $Image,
    [string] $Printer = 'Brother QL-800',
    # The loaded DK roll: 62, 50, 38 or 29 mm continuous tape, or 62x100 or 29x90 mm die-cut labels.
    [ValidateSet('62', '62x100', '50', '38', '29', '29x90')] [string] $Media = '62',
    [double] $FeedMarginMm = 3,
    [string] $PdfFile,
    [switch] $DryRun
)

$ErrorActionPreference = 'Stop'
Add-Type -AssemblyName System.Drawing
Add-Type -AssemblyName ReachFramework
Add-Type -AssemblyName System.Printing
Add-Type -Namespace FiducialLabels -Name Native -MemberDefinition @'
[DllImport("kernel32.dll")] public static extern System.IntPtr GlobalLock(System.IntPtr h);
[DllImport("kernel32.dll")] public static extern bool GlobalUnlock(System.IntPtr h);
[DllImport("kernel32.dll")] public static extern System.IntPtr GlobalFree(System.IntPtr h);
'@
$ImageDpi = 300.0
# Paper codes of the Brother QL-800 driver (1.10.1). That driver ignores a custom size
# (it falls back to 29 x 90 mm landscape), so the roll's own code is selected;
# continuous tape takes its length from the page.
$PaperCodes = @{ '62' = 259; '62x100' = 275; '50' = 262; '38' = 264; '29' = 258; '29x90' = 271 }
$ContinuousTape = @('62', '50', '38', '29')

function ConvertTo-Hundredths([double] $Mm) { [int][math]::Round($Mm / 25.4 * 100) }

function New-LabelPage([System.Drawing.Image] $Bitmap) {
    $page = $settings.DefaultPageSettings.Clone()
    $page.Landscape = $false
    $page.Margins = New-Object System.Drawing.Printing.Margins(0, 0, 0, 0)
    if ($PdfFile) { return $page }
    $listed = $settings.PaperSizes | Where-Object { $_.RawKind -eq $PaperCodes[$Media] } | Select-Object -First 1
    if (-not $listed) { throw "The '$Printer' driver offers no paper code $($PaperCodes[$Media]) for the $Media roll." }
    if ($ContinuousTape -contains $Media) {
        $lengthMm = $Bitmap.Height / $ImageDpi * 25.4 + 2 * $FeedMarginMm + 1
        $paper = New-Object System.Drawing.Printing.PaperSize($listed.PaperName, $listed.Width, (ConvertTo-Hundredths $lengthMm))
        $paper.RawKind = $listed.RawKind
        $page.PaperSize = $paper
    }
    else {
        $page.PaperSize = $listed
    }
    return $page
}

# What the driver makes of the page: its print ticket's media, orientation and resolution.
function Get-DriverPage([System.Drawing.Printing.PageSettings] $Page) {
    $handle = $settings.GetHdevmode($Page)
    try {
        $pointer = [FiducialLabels.Native]::GlobalLock($handle)
        $size = [Runtime.InteropServices.Marshal]::ReadInt16($pointer, 68) + [Runtime.InteropServices.Marshal]::ReadInt16($pointer, 70)
        $devmode = New-Object byte[] $size
        [Runtime.InteropServices.Marshal]::Copy($pointer, $devmode, 0, $size)
        [void][FiducialLabels.Native]::GlobalUnlock($handle)
    }
    finally {
        [void][FiducialLabels.Native]::GlobalFree($handle)
    }
    $converter = New-Object System.Printing.Interop.PrintTicketConverter($settings.PrinterName, 1)
    $xml = New-Object Xml.XmlDocument
    $xml.Load($converter.ConvertDevModeToPrintTicket($devmode).GetXmlStream())
    $ns = New-Object Xml.XmlNamespaceManager($xml.NameTable)
    $ns.AddNamespace('psf', 'http://schemas.microsoft.com/windows/2003/08/printing/printschemaframework')
    $option = { param($feature) $xml.SelectSingleNode("//psf:Feature[substring-after(@name, ':')='$feature']/psf:Option", $ns) }
    $number = { param($node, $name)
        $value = $node.SelectSingleNode("psf:ScoredProperty[substring-after(@name, ':')='$name']/psf:Value", $ns)
        if (-not $value) {
            $value = $xml.SelectSingleNode("//psf:ParameterInit[substring-after(@name, ':')='PageMediaSize$name']/psf:Value", $ns)
        }
        if ($value) { [double]$value.InnerText } else { [double]::NaN }
    }
    $media = & $option 'PageMediaSize'
    $resolution = & $option 'PageResolution'
    '{0} {1:N1} x {2:N1} mm, {3}, {4}x{5} dpi' -f ($media.name -replace '^.*:', ''), ((& $number $media 'MediaSizeWidth') / 1000),
        ((& $number $media 'MediaSizeHeight') / 1000), ((& $option 'PageOrientation').name -replace '^.*:', ''),
        (& $number $resolution 'ResolutionX'), (& $number $resolution 'ResolutionY')
}

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

foreach ($path in $Image) {
    $full = (Resolve-Path -LiteralPath $path).Path
    $name = [IO.Path]::GetFileName($full)
    $bitmap = [System.Drawing.Image]::FromFile($full)
    try {
        $widthIn = $bitmap.Width / $ImageDpi
        $heightIn = $bitmap.Height / $ImageDpi
        $page = New-LabelPage $bitmap
        $measure = $settings.CreateMeasurementGraphics($page)
        try {
            $measure.PageUnit = [System.Drawing.GraphicsUnit]::Inch
            $area = $measure.VisibleClipBounds
            $dpiX, $dpiY = $measure.DpiX, $measure.DpiY
        }
        finally {
            $measure.Dispose()
        }
        '{0}: {1} x {2} dots = {3:N2} x {4:N2} mm' -f $name, $bitmap.Width, $bitmap.Height, ($widthIn * 25.4), ($heightIn * 25.4)
        '  driver page: {0}; printable {1:N2} x {2:N2} mm at {3}x{4} dpi' -f (Get-DriverPage $page), ($area.Width * 25.4), ($area.Height * 25.4), $dpiX, $dpiY
        if (($dpiX % $ImageDpi) -ne 0 -or ($dpiY % $ImageDpi) -ne 0) {
            throw "The driver renders at $($dpiX)x$($dpiY) dpi, not a multiple of 300; the cells would be uneven. Nothing printed."
        }
        $slack = 0.5 / $ImageDpi
        if ($widthIn -gt $area.Width + $slack -or $heightIn -gt $area.Height + $slack) {
            throw 'The image does not fit the printable area; check -Media. Nothing printed.'
        }
        if ($DryRun) { continue }

        $doc = New-Object System.Drawing.Printing.PrintDocument
        $doc.PrinterSettings = $settings
        $doc.DefaultPageSettings = $page
        $doc.DocumentName = "Fiducial label $name"
        $doc.PrintController = New-Object System.Drawing.Printing.StandardPrintController
        $doc.OriginAtMargins = $false
        $doc.add_PrintPage({
            param($sender, $e)
            $g = $e.Graphics
            $g.PageUnit = [System.Drawing.GraphicsUnit]::Inch
            $g.InterpolationMode = [System.Drawing.Drawing2D.InterpolationMode]::NearestNeighbor
            $g.PixelOffsetMode = [System.Drawing.Drawing2D.PixelOffsetMode]::Half
            # The origin is the printable area's corner. Centre the image in that area and snap
            # its corner to a whole device dot so each image dot lands on printer dots.
            $clip = $g.VisibleClipBounds
            $x = [math]::Round(($clip.X + ($clip.Width - $widthIn) / 2) * $g.DpiX) / $g.DpiX
            $y = [math]::Round(($clip.Y + ($clip.Height - $heightIn) / 2) * $g.DpiY) / $g.DpiY
            $g.DrawImage($bitmap, (New-Object System.Drawing.RectangleF([single]$x, [single]$y, [single]$widthIn, [single]$heightIn)))
            Write-Host ('  printed at {0}x{1} dpi, image corner {2:N2}, {3:N2} mm inside the printable corner' -f $g.DpiX, $g.DpiY, ($x * 25.4), ($y * 25.4))
            $e.HasMorePages = $false
        })
        $doc.Print()
    }
    finally {
        $bitmap.Dispose()
    }
}
