"use client";

import React, { useEffect, useRef } from "react";
import { MapContainer, TileLayer, Marker, Popup, Polyline, useMap, useMapEvents } from "react-leaflet";
import "leaflet/dist/leaflet.css";
import L from "leaflet";
import { Navigation as NavIcon } from "lucide-react";

delete (L.Icon.Default.prototype as any)._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon-2x.png",
  iconUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-icon.png",
  shadowUrl: "https://unpkg.com/leaflet@1.7.1/dist/images/marker-shadow.png",
});

// Custom Car Marker Icon (SVG)
const carIcon = new L.DivIcon({
  className: "bg-transparent border-none",
  html: `<div class="relative flex items-center justify-center w-12 h-12">
           <div class="absolute w-8 h-8 bg-blue-500 rounded-full opacity-30 animate-ping"></div>
           <div class="relative w-4 h-4 bg-blue-500 rounded-full border-2 border-white shadow-[0_0_15px_rgba(59,130,246,0.8)]"></div>
         </div>`,
  iconSize: [48, 48],
  iconAnchor: [24, 24],
});

// Destination Marker Icon
const destIcon = new L.DivIcon({
  className: "bg-transparent border-none",
  html: `<div class="w-4 h-4 bg-red-500 rounded-full border-2 border-white shadow-[0_0_15px_rgba(239,68,68,0.8)]"></div>`,
  iconSize: [16, 16],
  iconAnchor: [8, 8],
});

interface NavState {
  lat: number;
  lng: number;
  target_lat: number;
  target_lng: number;
  heading: number;
  speed: number;
  steering: number;
}

// Moves the map and marker based on true physics backend coordinates
function MapDriver({ nav }: { nav: NavState | null }) {
  const map = useMap();
  const markerRef = useRef<L.Marker>(null);

  useEffect(() => {
    if (nav) {
      // Smoothly pan map to current location
      map.setView([nav.lat, nav.lng], map.getZoom(), { animate: true, duration: 0.5 });
      
      if (markerRef.current) {
        markerRef.current.setLatLng([nav.lat, nav.lng]);
        // Leaflet doesn't natively support marker rotation in the standard Marker,
        // but we can hack the DOM element if needed. For now, panning is the main focus.
      }
    }
  }, [nav, map]);

  if (!nav) return null;

  return (
    <Marker ref={markerRef} position={[nav.lat, nav.lng]} icon={carIcon}>
      <Popup className="bg-black/80 backdrop-blur border border-white/10 text-white rounded-lg p-2">
        <div className="text-xs font-semibold">DDS Autopilot Active</div>
        <div className="text-[10px] text-gray-400 mt-1">Speed: {Math.round(nav.speed)} km/h</div>
      </Popup>
    </Marker>
  );
}

// Allows clicking the map to set a new destination
function ClickHandler({ onSetDestination }: { onSetDestination: (lat: number, lng: number) => void }) {
  useMapEvents({
    click(e) {
      onSetDestination(e.latlng.lat, e.latlng.lng);
    },
  });
  return null;
}

interface MapPanelProps {
  navState: NavState | null;
  onSetDestination: (lat: number, lng: number) => void;
}

export default function MapPanel({ navState, onSetDestination }: MapPanelProps) {
  const darkMatterUrl = "https://{s}.basemaps.cartocdn.com/dark_all/{z}/{x}/{y}{r}.png";
  const attribution = '&copy; <a href="https://carto.com/attributions">CARTO</a>';

  // Default to SF if no nav state yet
  const centerLat = navState?.lat ?? 37.7749;
  const centerLng = navState?.lng ?? -122.4194;

  return (
    <div className="absolute inset-0 z-0 cursor-crosshair">
      <MapContainer 
        center={[centerLat, centerLng]} 
        zoom={16} 
        style={{ height: "100%", width: "100%" }}
        zoomControl={false}
        attributionControl={false}
      >
        <TileLayer url={darkMatterUrl} attribution={attribution} />
        <MapDriver nav={navState} />
        <ClickHandler onSetDestination={onSetDestination} />
        
        {/* Destination Marker & Route Polyline */}
        {navState && (
          <>
            <Marker position={[navState.target_lat, navState.target_lng]} icon={destIcon} />
            <Polyline 
              positions={[
                [navState.lat, navState.lng],
                [navState.target_lat, navState.target_lng]
              ]} 
              color="#3b82f6" 
              weight={3} 
              dashArray="10, 10" 
              opacity={0.5} 
            />
          </>
        )}
      </MapContainer>
      
      {/* Map UI Overlay */}
      <div className="absolute top-6 left-6 z-10 flex gap-4">
        <div className="bg-[#111111]/80 backdrop-blur-xl border border-white/10 rounded-full px-5 py-3 flex items-center shadow-2xl w-80">
          <NavIcon className="w-4 h-4 text-blue-400 mr-3" />
          <div className="flex flex-col">
            <span className="text-white text-sm font-medium">Click map to set destination</span>
            <span className="text-xs text-gray-500">
              {navState ? `${navState.target_lat.toFixed(4)}, ${navState.target_lng.toFixed(4)}` : "Waiting for GPS..."}
            </span>
          </div>
        </div>
      </div>
    </div>
  );
}
