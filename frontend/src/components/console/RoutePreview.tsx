"use client";

import { useEffect, useMemo, useRef, useState } from "react";
import type * as Leaflet from "leaflet";
import { useSimulationStore } from "../../store/useSimulationStore";

type MapLayers = {
  api: typeof Leaflet;
  map: Leaflet.Map;
  path: Leaflet.Polyline;
  outline: Leaflet.Polyline;
  vehicle: Leaflet.CircleMarker;
  start: Leaflet.Marker;
  destination: Leaflet.Marker;
  tiles: Leaflet.TileLayer;
};

export function RoutePreview({onPlanJourney}: {onPlanJourney?:()=>void}) {
  const container = useRef<HTMLDivElement>(null);
  const layers = useRef<MapLayers | null>(null);
  const follow = useRef(false);
  const [following, setFollowing] = useState(false);
  const [ready, setReady] = useState(false);
  const [unavailable, setUnavailable] = useState(false);
  const points = useSimulationStore((s) => s.routeWaypoints);
  const validPoints = useMemo(() => points.filter(([lat, lng]) =>
    Number.isFinite(lat) && Number.isFinite(lng) && Math.abs(lat) <= 90 && Math.abs(lng) <= 180), [points]);
  const position = useSimulationStore((s) => s.ego?.geographic_position);

  useEffect(() => {
    let disposed = false;
    let resize: ResizeObserver | undefined;
    let map: Leaflet.Map | undefined;
    import("leaflet").then((api) => {
      if (disposed || !container.current) return;
      map = api.map(container.current, {
        zoomControl: false,
        scrollWheelZoom: false,
        attributionControl: true,
      }).setView([37.63, -122.41], 13);
      map.attributionControl.setPrefix(false);
      // Zoom controls live in the compact navigation menu.
      const tiles = api.tileLayer(
        process.env.NEXT_PUBLIC_MAP_TILE_URL ||
          "https://tile.openstreetmap.org/{z}/{x}/{y}.png",
        {
          maxZoom: 19,
          attribution: process.env.NEXT_PUBLIC_MAP_ATTRIBUTION ||
            '&copy; <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noreferrer">OpenStreetMap</a> contributors',
        },
      );
      const failedTiles = new Set<HTMLElement>();
      tiles.on("tileerror", (event) => {
        failedTiles.add(event.tile);
        if (!disposed) setUnavailable(true);
      });
      tiles.on("tileload", (event) => {
        failedTiles.delete(event.tile);
        if (!disposed) setUnavailable(failedTiles.size > 0);
      });
      tiles.on("tileunload", (event) => {
        failedTiles.delete(event.tile);
        if (!disposed) setUnavailable(failedTiles.size > 0);
      });
      tiles.addTo(map);
      const pin = (label: string, className: string) => api.marker([0, 0], {
        icon: api.divIcon({ className: `route-map-pin ${className}`, html: `<i aria-hidden="true"></i>`,
          iconSize: [18, 24], iconAnchor: [9, label === "Start" ? 9 : 24] }), opacity: 0,
        keyboard: true, title: label,
      }).addTo(map!);
      const start = pin("Start", "route-map-start");
      const destination = pin("Destination", "route-map-destination");
      map.on("dragstart", () => { follow.current = false; setFollowing(false); });
      const outline = api.polyline([], {
        color: "#ffffff", weight: 0, opacity: 0, interactive: false,
      }).addTo(map);
      const path = api.polyline([], {
        color: "#263d72", weight: 4, opacity: 1, interactive: false,
      }).addTo(map);
      const vehicle = api.circleMarker([37.63, -122.41], {
        radius: 3, color: "#263d72", weight: 1, fillColor: "#263d72",
        fillOpacity: 1, opacity: 0,
      }).addTo(map);
      vehicle.bindTooltip("Simulated vehicle");
      layers.current = { api, map, path, outline, vehicle, start, destination, tiles };
      resize = new ResizeObserver(() => map?.invalidateSize());
      resize.observe(container.current);
      setReady(true);
    }).catch(() => { if (!disposed) setUnavailable(true); });
    return () => {
      disposed = true;
      resize?.disconnect();
      map?.remove();
      layers.current = null;
    };
  }, []);

  useEffect(() => {
    const current = layers.current;
    if (!ready || !current) return;
    const valid = validPoints;
    current.outline.setLatLngs(valid);
    current.path.setLatLngs(valid);
    current.start.setOpacity(valid.length > 1 ? 1 : 0);
    current.destination.setOpacity(valid.length > 1 ? 1 : 0);
    if (valid.length > 1) {
      current.start.setLatLng(valid[0]);
      current.destination.setLatLng(valid[valid.length - 1]);
      current.map.fitBounds(current.api.latLngBounds(valid), {
        paddingTopLeft: [44, 28], paddingBottomRight: [44, 46], maxZoom: 16, animate: false,
      });
    }
  }, [validPoints, ready]);

  useEffect(() => {
    const current = layers.current;
    if (!ready || !current) return;
    if (!position || !Number.isFinite(position.lat) || !Number.isFinite(position.lng) || Math.abs(position.lat) > 90 || Math.abs(position.lng) > 180) {
      current.vehicle.setStyle({ opacity: 0, fillOpacity: 0 });
      return;
    }
    current.vehicle.setLatLng([position.lat, position.lng]);
    current.vehicle.setStyle({ opacity: 1, fillOpacity: 1 });
    if (follow.current) current.map.panTo([position.lat, position.lng], { animate: false });
  }, [position, ready]);

  return (
    <div className="drive-map geographic-map" aria-label="Real world street map">
      <div ref={container} className="geographic-map-canvas" onClick={onPlanJourney} />
      {onPlanJourney && <button className="map-plan-button" aria-label="Open journey planner" onClick={onPlanJourney}>Plan a drive</button>}
      <span className="geographic-map-caption">Real roads · simulated vehicle</span>
      <details className="geographic-map-menu">
        <summary aria-label="Map controls">•••</summary>
      <div className="geographic-map-actions">
        <button type="button" aria-label="Show full route" onClick={() => {
          follow.current = false;
          setFollowing(false);
          const current = layers.current;
          if (current && validPoints.length > 1) current.map.fitBounds(current.api.latLngBounds(validPoints), { paddingTopLeft: [44, 28], paddingBottomRight: [44, 46], maxZoom: 16 });
        }}>Route</button>
        <button type="button" aria-label="Recenter map on vehicle" aria-pressed={following} onClick={() => {
          follow.current = !follow.current;
          setFollowing(follow.current);
          if (position && Number.isFinite(position.lat) && Number.isFinite(position.lng) && Math.abs(position.lat) <= 90 && Math.abs(position.lng) <= 180 && layers.current) layers.current.map.setView([position.lat, position.lng], 16);
        }}>{following ? "Following" : "Recenter"}</button>
        <button type="button" aria-label="Zoom in" onClick={() => layers.current?.map.zoomIn()}>+</button>
        <button type="button" aria-label="Zoom out" onClick={() => layers.current?.map.zoomOut()}>−</button>
      </div>
      </details>
      {unavailable && <span className="geographic-map-error" role="status">Map tiles unavailable <button type="button" onClick={() => {
        layers.current?.tiles.redraw();
      }}>Retry</button></span>}
    </div>
  );
}
