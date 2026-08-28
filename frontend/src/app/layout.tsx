import type { Metadata } from "next";
import { Outfit, Inter, JetBrains_Mono } from "next/font/google";
import "./globals.css";
import { CommandPalette } from "../components/CommandPalette";

const outfit = Outfit({ subsets: ["latin"], weight: ["400", "500", "600", "700", "800"], variable: "--font-outfit", display: "swap" });
const inter = Inter({ subsets: ["latin"], weight: ["300", "400", "500", "600", "700"], variable: "--font-inter", display: "swap" });
const jetbrains = JetBrains_Mono({ subsets: ["latin"], weight: ["400", "500", "600", "700"], variable: "--font-jetbrains", display: "swap" });

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
    <html lang="en" className={`dark ${outfit.variable} ${inter.variable} ${jetbrains.variable}`}>
      <body className="font-sans antialiased bg-background text-foreground">
        <CommandPalette />
        <main className="h-screen overflow-hidden">{children}</main>
      </body>
    </html>
  );
}
