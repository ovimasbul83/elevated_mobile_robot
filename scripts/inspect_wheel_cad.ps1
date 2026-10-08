param(
    [Parameter(Mandatory = $true)]
    [string]$SourceAssembly
)

$ErrorActionPreference = 'Stop'
if ([Environment]::OSVersion.Platform -ne [PlatformID]::Win32NT) {
    throw 'CAD import requires Windows and Autodesk Inventor. The simulation uses exported meshes and does not require Inventor.'
}
$source = (Resolve-Path -LiteralPath $SourceAssembly).Path
$workspace = Split-Path $PSScriptRoot -Parent
$outputDir = Join-Path $workspace 'outputs/wheel_cad'
New-Item -ItemType Directory -Path $outputDir -Force | Out-Null
$localSource = Join-Path $outputDir (Split-Path $source -Leaf)
Copy-Item -LiteralPath $source -Destination $localSource -Force
$inventor = $null
$document = $null
try {
    Write-Output 'Starting hidden Inventor CAD importer...'
    $inventor = New-Object -ComObject Inventor.Application
    $inventor.Visible = $false
    $inventor.SilentOperation = $true
    Write-Output 'Opening a workspace copy of the supplied wheel assembly...'
    $document = $inventor.Documents.Open($localSource, $false)
    $definition = $document.ComponentDefinition
    $stl = $inventor.ApplicationAddIns | Where-Object { $_.DisplayName -eq 'Translator: STL Export' } | Select-Object -First 1
    if ($null -eq $stl) {
        $inventor.ApplicationAddIns | ForEach-Object { $_.DisplayName } | Write-Output
        throw 'STL translator not found'
    }
    $stl.Activate()
    $context = $inventor.TransientObjects.CreateTranslationContext()
    $context.Type = 13059
    $options = $inventor.TransientObjects.CreateNameValueMap()
    $options.Add('ExportUnits', 6)
    $options.Add('ExportFileStructure', 0)
    $options.Add('OutputFileType', 0)
    $options.Add('Resolution', 1)
    $options.Add('ExportColor', $true)
    $exported = @{}
    $partIndex = 0
    $occurrenceInfo = @()
    foreach ($occurrence in $definition.Occurrences) {
        $part = $occurrence.Definition.Document
        $key = $part.DisplayName
        if (-not $exported.ContainsKey($key)) {
            $filename = 'part_{0:d2}.stl' -f $partIndex
            $partIndex++
            $medium = $inventor.TransientObjects.CreateDataMedium()
            $medium.FileName = Join-Path $outputDir $filename
            $stl.SaveCopyAs($part, $context, $options, $medium)
            if (-not (Test-Path -LiteralPath $medium.FileName)) { throw "STL export missing: $key" }
            $exported[$key] = $filename
        }
        $matrix = $occurrence.Transformation
        $transform = @(foreach ($row in 1..4) { foreach ($column in 1..4) { $matrix.Cell($row, $column) } })
        $occurrenceInfo += [ordered]@{
            Name = $occurrence.Name
            Mesh = $exported[$key]
            Transform = $transform
            MassKg = $occurrence.MassProperties.Mass
        }
    }
    $occurrenceInfo | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $outputDir 'mesh_instances.json')
    $bounds = $definition.RangeBox
    $info = [ordered]@{
        Name = $document.DisplayName
        DocumentType = $document.DocumentType
        MinimumCentimetres = @($bounds.MinPoint.X, $bounds.MinPoint.Y, $bounds.MinPoint.Z)
        MaximumCentimetres = @($bounds.MaxPoint.X, $bounds.MaxPoint.Y, $bounds.MaxPoint.Z)
        MassKg = $definition.MassProperties.Mass
        Occurrences = @($definition.Occurrences | ForEach-Object {
            $b = $_.RangeBox
            [ordered]@{
                Name = $_.Name
                MinimumCentimetres = @($b.MinPoint.X, $b.MinPoint.Y, $b.MinPoint.Z)
                MaximumCentimetres = @($b.MaxPoint.X, $b.MaxPoint.Y, $b.MaxPoint.Z)
                MassKg = $_.MassProperties.Mass
            }
        })
    }
    $info | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $outputDir 'assembly_info.json')
    Write-Output ("Imported {0} components; exported {1} unique meshes in metres." -f $definition.Occurrences.Count, $exported.Count)
    if ($definition.Occurrences.Count -eq 0) { throw 'No assembly components imported.' }
} finally {
    if ($null -ne $document) { $document.Close($true) }
    if ($null -ne $inventor) { $inventor.Quit() }
}
