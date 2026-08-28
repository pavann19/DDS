import type { Metadata } from "next";
import "./globals.css";
import { CommandPalette } from "../components/CommandPalette";

export const metadata: Metadata = {
  title: "DDS — Driving Decision System",
  description:
    "Operator console for the DDS autonomous-driving simulation: live World/Driver telemetry, perception, prediction, and safety-shield state.",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="dark">
      <body className="font-sans antialiased bg-background text-foreground">
        <CommandPalette />
        <main className="h-screen overflow-hidden">{children}</main>
      </body>
    </html>
  );
}
