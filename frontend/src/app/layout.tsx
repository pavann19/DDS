import type { Metadata } from "next";
import { Outfit, Inter, JetBrains_Mono } from "next/font/google";
import "./globals.css";
import { CommandPalette } from "../components/CommandPalette";

const outfit = Outfit({ subsets: ["latin"], weight: ["400", "500", "600", "700", "800"], variable: "--font-outfit", display: "swap" });
const inter = Inter({ subsets: ["latin"], weight: ["300", "400", "500", "600", "700"], variable: "--font-inter", display: "swap" });
const jetbrains = JetBrains_Mono({ subsets: ["latin"], weight: ["400", "500", "600", "700"], variable: "--font-jetbrains", display: "swap" });

export const metadata: Metadata = {
  title: "DDS Drive",
  description:
    "A clear view of your drive. Vehicle information, navigation, surroundings and safety status in one driving cockpit.",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className={`${outfit.variable} ${inter.variable} ${jetbrains.variable}`}>
      <body className="font-sans antialiased bg-background text-foreground">
        <CommandPalette />
        <div className="h-screen overflow-hidden">{children}</div>
      </body>
    </html>
  );
}
