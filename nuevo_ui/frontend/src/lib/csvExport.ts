/**
 * CSV Export Utility
 *
 * Generates CSV from time-series data and triggers browser download.
 */

export function downloadCSV(
  headers: string[],
  data: number[][],
  filename: string
) {
  // Transpose data: [[time...], [series1...], [series2...]] -> [[time, s1, s2], ...]
  const rows: number[][] = []
  const numRows = data[0]?.length || 0

  for (let i = 0; i < numRows; i++) {
    const row: number[] = []
    for (let j = 0; j < data.length; j++) {
      row.push(data[j][i])
    }
    rows.push(row)
  }

  // Generate CSV string
  const csvLines = [
    headers.join(','),
    ...rows.map((row) => row.join(',')),
  ]
  const csvContent = csvLines.join('\n')

  // Trigger download
  const blob = new Blob([csvContent], { type: 'text/csv;charset=utf-8;' })
  const url = URL.createObjectURL(blob)
  const link = document.createElement('a')
  link.href = url
  link.download = filename
  link.style.display = 'none'
  document.body.appendChild(link)
  link.click()
  document.body.removeChild(link)
  URL.revokeObjectURL(url)
}
