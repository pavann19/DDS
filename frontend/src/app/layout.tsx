import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "DDS Autopilot Dashboard",
  description: "Driving Decision Strategy — Real-time telemetry dashboard powered by XGBoost and exhaustive feature optimization.",
};

import { CommandPalette } from "../components/CommandPalette";

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="dark">
      <body className="font-sans antialiased bg-background text-foreground flex">
        <CommandPalette />
        <main className="flex-1 h-screen overflow-hidden">
          {children}
        </main>
      </body>
    </html>
  );
}
