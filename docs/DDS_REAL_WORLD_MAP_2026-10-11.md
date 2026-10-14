# Real world map integration

The top-right map now uses Leaflet 1.9.4 and real OpenStreetMap raster tiles.
The decorative SVG street grid was removed. Vehicle coordinates and path
waypoints are projected geographically by Leaflet, with a dashed path and an
explicit simulated-vehicle label. Pan and zoom are enabled; wheel zoom is
disabled to avoid accidental changes. Attribution remains visible on mobile.
Only visible map tiles are requested, using normal browser caching. Tile
failures display an unavailable message rather than fabricated streets.

Provider overrides: `NEXT_PUBLIC_MAP_TILE_URL` and
`NEXT_PUBLIC_MAP_ATTRIBUTION`. The default online provider follows
https://operations.osmfoundation.org/policies/tiles/ and requires connectivity.

## Observed validation

- Browser loaded five OpenStreetMap tiles successfully on desktop.
- Desktop viewport and app width both measured 1440 CSS pixels; map right edge
  measured 1414 pixels, inside the viewport.
- Desktop and mobile screenshots inspected, including visible attribution.
- TypeScript and focused ESLint checks exited 0.
- Evidence: `output/playwright/cockpit-redesign/real-map-verified.png` and
  `real-map-mobile-verified.png`.

## Other issues discovered

**Follow-up:** Both issues below were addressed later on 2026-10-11. See
`DDS_REAL_ROADS_SECURITY_FIX_2026-10-11.md` for changes and validation limits.

- `SimulationSession._init_default_route` uses `synthetic_highway_route`.
  The real basemap exposes this generated path crossing areas without roads,
  including water. This map change does not convert that backend geometry into
  real road routing. Destination routing uses OSRM, but the default route and
  scenario reset behavior need a separate road-geometry correction.
- `npm audit` reports 15 dependency vulnerabilities, including a critical
  advisory for Next.js. Neither Leaflet nor its types appears in the advisory
  package list. Full audit: `output/frontend-audit-map.json`. No blanket
  dependency upgrades were applied.
