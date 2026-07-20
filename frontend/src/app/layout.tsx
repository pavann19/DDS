import type { Metadata } from "next";
import { Inter } from "next/font/google";
import "./globals.css";
import Sidebar from "./components/Sidebar";

const inter = Inter({
  variable: "--font-geist-sans",
  subsets: ["latin"],
  weight: ["300", "400", "500", "600", "700"],
});

export const metadata: Metadata = {
  title: "DDS Autopilot Dashboard",
  description: "Driving Decision Strategy — Real-time telemetry dashboard powered by XGBoost and exhaustive feature optimization.",
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="en" className="dark">
      <body
        className={`${inter.variable} font-sans antialiased bg-[#050510] text-gray-100 flex`}
      >
        <main className="flex-1 h-screen overflow-hidden">
          {children}
        </main>
      </body>
    </html>
  );
}
